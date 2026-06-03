#!/usr/bin/env python3
"""
GO2 Perception Node
YOLOv8-based human detection on RGB-D stream from Intel RealSense D435i.
Associates 2D bounding boxes with depth measurements to produce 3D human poses.
Publishes DetectedHuman array for the intent grounding node.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from ultralytics import YOLO

# Custom message — see go2_msgs/msg/DetectedHuman.msg
from go2_msgs.msg import DetectedHuman, DetectedHumanArray, PipelineMetrics

_NS_TO_MS = 1e-6  # nanoseconds to milliseconds


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("person_class_id", 0)
        self.declare_parameter("max_depth_m", 8.0)
        self.declare_parameter("depth_roi_fraction", 0.3)
        self.declare_parameter("enable_metrics", True)

        self.model_path = self.get_parameter("model_path").value
        self.conf_thresh = self.get_parameter("confidence_threshold").value
        self.person_cls = self.get_parameter("person_class_id").value
        self.max_depth = self.get_parameter("max_depth_m").value
        self.depth_roi = self.get_parameter("depth_roi_fraction").value
        self._enable_metrics = self.get_parameter("enable_metrics").value

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.camera_info = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, qos
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.depth_callback, qos
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.info_callback, 10
        )

        # Publishers
        self.humans_pub = self.create_publisher(DetectedHumanArray, "/go2/detected_humans", 10)
        self.vis_pub = self.create_publisher(Image, "/go2/perception/visualization", 10)
        self.metrics_pub = self.create_publisher(
            PipelineMetrics, "/go2/perception/metrics", 10
        )

        self.latest_depth = None
        self._frame_id: int = 0
        self.get_logger().info(f"PerceptionNode ready. Model: {self.model_path}")

    def info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def rgb_callback(self, msg: Image):
        if self.camera_info is None or self.latest_depth is None:
            return

        t_cb_start = time.perf_counter_ns()

        # Stage: cv_bridge decode
        t0 = time.perf_counter_ns()
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        t1 = time.perf_counter_ns()

        # Stage: YOLO inference
        results = self.model(rgb, verbose=False)[0]
        t2 = time.perf_counter_ns()

        human_array = DetectedHumanArray()
        human_array.header = Header()
        human_array.header.stamp = self.get_clock().now().to_msg()
        human_array.header.frame_id = "camera_color_optical_frame"

        depth_img = self.latest_depth.copy()
        vis_img = rgb.copy()

        # Per-box accumulators for depth sampling and back-projection
        depth_sampling_ns: int = 0
        backproject_ns: int = 0

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            if cls_id != self.person_cls or conf < self.conf_thresh:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx_bb = (x1 + x2) // 2
            cy_bb = (y1 + y2) // 2

            # Stage: depth sampling (ROI extraction + median) — accumulate across boxes
            t_ds0 = time.perf_counter_ns()
            roi_w = max(1, int((x2 - x1) * self.depth_roi))
            roi_h = max(1, int((y2 - y1) * self.depth_roi))
            rx1 = max(0, cx_bb - roi_w // 2)
            rx2 = min(depth_img.shape[1], cx_bb + roi_w // 2)
            ry1 = max(0, cy_bb - roi_h // 2)
            ry2 = min(depth_img.shape[0], cy_bb + roi_h // 2)

            depth_roi_img = depth_img[ry1:ry2, rx1:rx2].astype(np.float32)
            depth_roi_img = depth_roi_img[depth_roi_img > 0]  # Remove zeros
            t_ds1 = time.perf_counter_ns()
            depth_sampling_ns += t_ds1 - t_ds0

            if depth_roi_img.size == 0:
                continue

            # Median depth in mm -> meters (still depth-sampling stage)
            t_ds2 = time.perf_counter_ns()
            depth_m = float(np.median(depth_roi_img)) / 1000.0
            t_ds3 = time.perf_counter_ns()
            depth_sampling_ns += t_ds3 - t_ds2

            if depth_m <= 0 or depth_m > self.max_depth:
                continue

            # Stage: back-projection (3 multiplies) — accumulate across boxes
            t_bp0 = time.perf_counter_ns()
            X = (cx_bb - self.cx) * depth_m / self.fx
            Y = (cy_bb - self.cy) * depth_m / self.fy
            Z = depth_m
            t_bp1 = time.perf_counter_ns()
            backproject_ns += t_bp1 - t_bp0

            human = DetectedHuman()
            human.confidence = conf
            human.pose.position.x = X
            human.pose.position.y = Y
            human.pose.position.z = Z
            human.pose.orientation.w = 1.0
            human.bbox_x1 = x1
            human.bbox_y1 = y1
            human.bbox_x2 = x2
            human.bbox_y2 = y2
            human_array.humans.append(human)

        t3 = time.perf_counter_ns()

        # Stage: visualization (cv2 drawing for all accepted boxes, then cv2_to_imgmsg)
        for human in human_array.humans:
            x1, y1, x2, y2 = human.bbox_x1, human.bbox_y1, human.bbox_x2, human.bbox_y2
            cx_bb = (x1 + x2) // 2
            cy_bb = (y1 + y2) // 2
            color = (0, 255, 80)
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)
            label = f"person {human.confidence:.2f} | {human.pose.position.z:.2f}m"
            cv2.putText(vis_img, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.circle(vis_img, (cx_bb, cy_bb), 4, (0, 255, 255), -1)

        vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
        vis_msg.header = human_array.header
        t4 = time.perf_counter_ns()

        # Stage: publish
        self.humans_pub.publish(human_array)
        self.vis_pub.publish(vis_msg)
        t5 = time.perf_counter_ns()

        t_cb_end = time.perf_counter_ns()

        self._frame_id += 1

        if self._enable_metrics:
            m = PipelineMetrics()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "camera_color_optical_frame"
            m.node_name = "perception"
            m.frame_id = self._frame_id
            m.cv_bridge_rgb_ms = (t1 - t0) * _NS_TO_MS
            m.yolo_inference_ms = (t2 - t1) * _NS_TO_MS
            m.depth_sampling_ms = depth_sampling_ns * _NS_TO_MS
            m.backproject_ms = backproject_ns * _NS_TO_MS
            m.visualization_ms = (t4 - t3) * _NS_TO_MS
            m.publish_ms = (t5 - t4) * _NS_TO_MS
            # safety fields: 0.0 (default float64)
            m.safety_obstacle_ms = 0.0
            m.safety_stair_ms = 0.0
            m.safety_drop_ms = 0.0
            m.safety_passage_ms = 0.0
            m.safety_total_ms = 0.0
            m.total_ms = (t_cb_end - t_cb_start) * _NS_TO_MS
            self.metrics_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
