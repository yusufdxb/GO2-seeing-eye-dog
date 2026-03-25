#!/usr/bin/env python3
"""
GO2 Intent Grounding Node
Fuses audio bearing (from audio_perception_node) with visual human detections
(from perception_node) to identify the most likely caller and confirm intent
before initiating navigation.

Confidence gating: both audio and visual thresholds must be met before
the confirmed target is published. This prevents false triggers.
"""

import math
import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

from go2_msgs.msg import DetectedHumanArray, ConfirmedTarget
from go2_intent_grounding.fusion import compute_fused_score


class IntentGroundingNode(Node):
    def __init__(self):
        super().__init__("intent_grounding_node")

        # Parameters
        self.declare_parameter("audio_weight", 0.4)
        self.declare_parameter("visual_weight", 0.6)
        self.declare_parameter("bearing_tolerance_deg", 25.0)
        self.declare_parameter("min_confidence_threshold", 0.65)
        self.declare_parameter("confirmation_frames", 5)
        self.declare_parameter("audio_timeout_sec", 2.0)

        self.audio_w = self.get_parameter("audio_weight").value
        self.visual_w = self.get_parameter("visual_weight").value
        self.bearing_tol = math.radians(self.get_parameter("bearing_tolerance_deg").value)
        self.min_conf = self.get_parameter("min_confidence_threshold").value
        self.confirm_frames = self.get_parameter("confirmation_frames").value
        self.audio_timeout = self.get_parameter("audio_timeout_sec").value

        # State
        self.latest_bearing_rad = None
        self.bearing_timestamp = None
        self.consecutive_confirmations = 0
        self.target_locked = False
        self.locked_target_pose = None

        # Subscribers
        self.bearing_sub = self.create_subscription(
            Float32, "/go2/audio/bearing_deg", self.bearing_callback, 10
        )
        self.humans_sub = self.create_subscription(
            DetectedHumanArray, "/go2/detected_humans", self.humans_callback, 10
        )
        self.voice_sub = self.create_subscription(
            String, "/go2/voice_command", self.voice_callback, 10
        )

        # Publishers
        self.target_pub = self.create_publisher(ConfirmedTarget, "/go2/confirmed_target", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.state_pub = self.create_publisher(String, "/go2/grounding_state", 10)

        # TF2 — for transforming camera-frame detections into the map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("IntentGroundingNode ready.")

    def bearing_callback(self, msg: Float32):
        self.latest_bearing_rad = math.radians(msg.data)
        self.bearing_timestamp = self.get_clock().now()

    def voice_callback(self, msg: String):
        command = msg.data.lower().strip()
        if command in ("come here", "come", "here", "over here"):
            self.get_logger().info(f"Voice command received: '{command}' — grounding intent")
            # Reset lock to allow re-identification on new call
            self.target_locked = False
            self.consecutive_confirmations = 0
        elif command in ("stop", "wait", "stay"):
            self.get_logger().info("Stop command — canceling navigation")
            self._publish_state("STOPPED")
            self.target_locked = False

    def humans_callback(self, msg: DetectedHumanArray):
        if not msg.humans:
            return

        # Check if audio bearing is fresh
        audio_valid = False
        if self.latest_bearing_rad is not None and self.bearing_timestamp is not None:
            age = (self.get_clock().now() - self.bearing_timestamp).nanoseconds / 1e9
            audio_valid = age < self.audio_timeout

        best_score = -1.0
        best_human = None

        for human in msg.humans:
            human_angle = math.atan2(human.pose.position.x, human.pose.position.z)
            fused_score = compute_fused_score(
                visual_score=human.confidence,
                human_angle_rad=human_angle,
                audio_bearing_rad=self.latest_bearing_rad,
                bearing_tol_rad=self.bearing_tol,
                audio_weight=self.audio_w,
                visual_weight=self.visual_w,
                audio_valid=audio_valid,
            )
            if fused_score > best_score:
                best_score = fused_score
                best_human = human

        if best_human is None or best_score < self.min_conf:
            self.consecutive_confirmations = 0
            self._publish_state("SEARCHING")
            return

        self.consecutive_confirmations += 1
        self.get_logger().debug(
            f"Candidate score={best_score:.2f} confirmations={self.consecutive_confirmations}"
        )

        if self.consecutive_confirmations >= self.confirm_frames and not self.target_locked:
            self.target_locked = True
            self.locked_target_pose = best_human.pose

            confirmed = ConfirmedTarget()
            confirmed.header = msg.header
            confirmed.pose = best_human.pose
            confirmed.confidence = float(best_score)
            self.target_pub.publish(confirmed)

            # Publish Nav2 goal — transform from camera frame to map frame via TF2
            camera_ps = PoseStamped()
            camera_ps.header = msg.header
            camera_ps.pose = best_human.pose
            try:
                map_ps = self.tf_buffer.transform(
                    camera_ps,
                    "map",
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                self.goal_pub.publish(map_ps)
            except tf2_ros.TransformException as e:
                self.get_logger().warn(
                    f"TF lookup camera→map failed: {e}. Goal not published."
                )

            self._publish_state("TARGET_LOCKED")
            self.get_logger().info(
                f"Target confirmed! Distance: {best_human.pose.position.z:.2f}m | "
                f"Score: {best_score:.2f}"
            )

    def _publish_state(self, state: str):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntentGroundingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
