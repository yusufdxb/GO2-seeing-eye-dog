#!/usr/bin/env python3
"""
GO2 Safety Monitor Node
Analyzes depth images for safety-critical obstacles:
  - Stairs (upward and downward)
  - Curbs and ground-level drops
  - Narrow passages (width < robot clearance)
  - Close obstacles in forward path

Publishes safety alerts that the Nav2 behavior tree responds to.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from go2_msgs.msg import PipelineMetrics, SafetyAlert

# Robot physical parameters
ROBOT_WIDTH_M = 0.35          # GO2 shoulder width
MIN_PASSAGE_WIDTH_M = 0.55    # Minimum safe passage (robot + clearance)
MAX_STEP_HEIGHT_M = 0.08      # Maximum safe step height for the GO2
STOP_DISTANCE_M = 0.4         # Emergency stop distance
SLOWDOWN_DISTANCE_M = 1.0     # Begin slowing at this distance

_NS_TO_MS = 1e-6  # nanoseconds to milliseconds


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__("safety_monitor_node")

        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("floor_band_fraction", 0.15)
        self.declare_parameter("stair_gradient_threshold", 0.04)
        self.declare_parameter("enable_metrics", True)

        self.max_depth = self.get_parameter("max_depth_m").value
        self.floor_band = self.get_parameter("floor_band_fraction").value
        self.stair_grad_thresh = self.get_parameter("stair_gradient_threshold").value
        self._enable_metrics = self.get_parameter("enable_metrics").value

        self.bridge = CvBridge()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.depth_callback, qos
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self.info_callback, 10
        )

        self.alert_pub = self.create_publisher(SafetyAlert, "/go2/safety_alert", 10)
        self.state_pub = self.create_publisher(String, "/go2/safety_state", 10)
        self.vis_pub = self.create_publisher(Image, "/go2/safety/visualization", 10)
        self.metrics_pub = self.create_publisher(
            PipelineMetrics, "/go2/safety_monitor/metrics", 10
        )

        # Camera intrinsics — populated by CameraInfo callback
        self.fx = None

        self._frame_id: int = 0
        self.get_logger().info("SafetyMonitorNode ready.")

    def info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.get_logger().info(f"Camera intrinsics received: fx={self.fx:.1f}")

    def depth_callback(self, msg: Image):
        t_cb_start = time.perf_counter_ns()

        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_m = depth_raw.astype(np.float32) / 1000.0

        h, w = depth_m.shape
        alerts = []
        vis = cv2.cvtColor(
            cv2.normalize(
                np.clip(depth_m, 0, self.max_depth),
                None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U,
            ),
            cv2.COLOR_GRAY2BGR,
        )

        # ── 1. Forward obstacle check ──────────────────────────────────
        t_obs0 = time.perf_counter_ns()
        center_col_start = w // 3
        center_col_end = 2 * w // 3
        row_start = h // 3
        row_end = 2 * h // 3
        roi = depth_m[row_start:row_end, center_col_start:center_col_end]
        valid = roi[(roi > 0) & (roi < self.max_depth)]

        if valid.size > 0:
            min_dist = float(np.percentile(valid, 5))
            if min_dist < STOP_DISTANCE_M:
                alerts.append(("EMERGENCY_STOP", min_dist, "Obstacle within stop distance"))
                cv2.rectangle(vis, (center_col_start, row_start), (center_col_end, row_end), (0, 0, 255), 2)
            elif min_dist < SLOWDOWN_DISTANCE_M:
                alerts.append(("SLOWDOWN", min_dist, "Obstacle within slowdown distance"))
                cv2.rectangle(vis, (center_col_start, row_start), (center_col_end, row_end), (0, 165, 255), 2)
        t_obs1 = time.perf_counter_ns()

        # ── 2. Stair detection (horizontal gradient in floor band) ──────
        t_stair0 = time.perf_counter_ns()
        floor_row_start = int(h * (1.0 - self.floor_band))
        floor_band_depth = depth_m[floor_row_start:, :]
        valid_floor = np.where(
            (floor_band_depth > 0) & (floor_band_depth < self.max_depth),
            floor_band_depth, np.nan,
        )
        if not np.all(np.isnan(valid_floor)):
            col_means = np.nanmean(valid_floor, axis=0)
            grad = np.abs(np.diff(col_means))
            grad = grad[~np.isnan(grad)]
            if grad.size > 0 and np.max(grad) > self.stair_grad_thresh:
                alerts.append(("STAIRS_DETECTED", float(np.nanmean(valid_floor)), "Stair-like depth gradient"))
                cv2.rectangle(vis, (0, floor_row_start), (w, h), (255, 0, 128), 2)
        t_stair1 = time.perf_counter_ns()

        # ── 3. Drop / curb detection ─────────────────────────────────────
        t_drop0 = time.perf_counter_ns()
        near_floor = depth_m[h - 20 : h, w // 3 : 2 * w // 3]
        far_floor = depth_m[h // 2 : h // 2 + 20, w // 3 : 2 * w // 3]
        near_valid = near_floor[(near_floor > 0) & (near_floor < self.max_depth)]
        far_valid = far_floor[(far_floor > 0) & (far_floor < self.max_depth)]

        if near_valid.size > 10 and far_valid.size > 10:
            near_med = float(np.median(near_valid))
            far_med = float(np.median(far_valid))
            drop = far_med - near_med
            if drop > MAX_STEP_HEIGHT_M * 2:  # conservative threshold
                alerts.append(("DROP_DETECTED", drop, f"Ground drop {drop:.2f}m"))
        t_drop1 = time.perf_counter_ns()

        # ── 4. Narrow passage detection ────────────────────────────────
        t_passage0 = time.perf_counter_ns()
        mid_row = depth_m[h // 2, :]
        close_mask = (mid_row > 0) & (mid_row < SLOWDOWN_DISTANCE_M)
        if np.any(close_mask):
            # Find largest gap without close obstacles
            gap = 0
            current_gap = 0
            for v in close_mask:
                if not v:
                    current_gap += 1
                    gap = max(gap, current_gap)
                else:
                    current_gap = 0
            if self.fx is not None and self.fx > 0:
                # Metric width at reference depth using pinhole projection: W = pixels * Z / fx
                gap_m = gap * SLOWDOWN_DISTANCE_M / self.fx
            else:
                # Fallback: approximate using 86° horizontal FOV (RealSense D435i default)
                gap_m = gap * (2.0 * np.tan(np.radians(43)) * SLOWDOWN_DISTANCE_M) / w
            if gap_m < MIN_PASSAGE_WIDTH_M:
                alerts.append(("NARROW_PASSAGE", gap_m, f"Passage width {gap_m:.2f}m"))
        t_passage1 = time.perf_counter_ns()

        t_safety_end = time.perf_counter_ns()

        # ── Publish ────────────────────────────────────────────────────
        if alerts:
            highest = max(alerts, key=lambda a: {
                "EMERGENCY_STOP": 4, "STAIRS_DETECTED": 3, "DROP_DETECTED": 3,
                "NARROW_PASSAGE": 2, "SLOWDOWN": 1
            }.get(a[0], 0))

            alert_msg = SafetyAlert()
            alert_msg.header.stamp = self.get_clock().now().to_msg()
            alert_msg.alert_type = highest[0]
            alert_msg.distance = highest[1]
            alert_msg.description = highest[2]
            self.alert_pub.publish(alert_msg)

            state_msg = String()
            state_msg.data = highest[0]
            self.state_pub.publish(state_msg)

            cv2.putText(vis, highest[0], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            state_msg = String()
            state_msg.data = "CLEAR"
            self.state_pub.publish(state_msg)

        vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        self.vis_pub.publish(vis_msg)

        t_cb_end = time.perf_counter_ns()

        self._frame_id += 1

        if self._enable_metrics:
            m = PipelineMetrics()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = ""
            m.node_name = "safety_monitor"
            m.frame_id = self._frame_id
            # perception fields: 0.0 (default float64)
            m.cv_bridge_rgb_ms = 0.0
            m.yolo_inference_ms = 0.0
            m.depth_sampling_ms = 0.0
            m.backproject_ms = 0.0
            m.visualization_ms = 0.0
            m.publish_ms = 0.0
            # safety stages
            m.safety_obstacle_ms = (t_obs1 - t_obs0) * _NS_TO_MS
            m.safety_stair_ms = (t_stair1 - t_stair0) * _NS_TO_MS
            m.safety_drop_ms = (t_drop1 - t_drop0) * _NS_TO_MS
            m.safety_passage_ms = (t_passage1 - t_passage0) * _NS_TO_MS
            m.safety_total_ms = (t_safety_end - t_cb_start) * _NS_TO_MS
            m.total_ms = (t_cb_end - t_cb_start) * _NS_TO_MS
            self.metrics_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
