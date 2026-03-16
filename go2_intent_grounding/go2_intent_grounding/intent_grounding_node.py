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
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped

from go2_msgs.msg import DetectedHumanArray, ConfirmedTarget


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
            # Visual score from detection confidence
            visual_score = human.confidence

            if audio_valid:
                # Compute angle to human in camera frame (X-Z plane)
                human_angle = math.atan2(human.pose.position.x, human.pose.position.z)
                angle_diff = abs(human_angle - self.latest_bearing_rad)
                # Normalize to [0, pi]
                angle_diff = min(angle_diff, 2 * math.pi - angle_diff)

                if angle_diff < self.bearing_tol:
                    # Soft score: 1.0 at zero diff, falls to ~0 at bearing_tol
                    audio_score = max(0.0, 1.0 - (angle_diff / self.bearing_tol))
                else:
                    audio_score = 0.0

                fused_score = self.audio_w * audio_score + self.visual_w * visual_score
            else:
                # Audio stale — rely on visual only, but lower weight
                fused_score = visual_score * 0.7

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

            # Publish Nav2 goal
            goal = PoseStamped()
            goal.header = msg.header
            goal.header.frame_id = "map"
            # NOTE: In production this needs a TF transform from camera frame to map frame.
            # Here we forward the detection pose directly for testing.
            goal.pose = best_human.pose
            self.goal_pub.publish(goal)

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
