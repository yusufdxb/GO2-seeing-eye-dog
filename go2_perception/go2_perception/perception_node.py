#!/usr/bin/env python3
"""
GO2 Perception Node
YOLOv8-based human detection on RGB-D stream from Intel RealSense D435i.
Associates 2D bounding boxes with depth measurements to produce 3D human poses.
Publishes DetectedHuman array for the intent grounding node.
"""

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
from go2_msgs.msg import DetectedHuman, DetectedHumanArray


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("person_class_id", 0)
        self.declare_parameter("max_depth_m", 8.0)
        self.declare_parameter("depth_roi_fraction", 0.3)

        self.model_path = self.get_parameter("model_path").value
        self.conf_thresh = self.get_parameter("confidence_threshold").value
        self.person_cls = self.get_parameter("person_class_id").value
        self.max_depth = self.get_parameter("max_depth_m").value
        self.depth_roi = self.get_parameter("depth_roi_fraction").value

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

        self.latest_depth = None
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

        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(rgb, verbose=False)[0]

        human_array = DetectedHumanArray()
        human_array.header = Header()
        human_array.header.stamp = self.get_clock().now().to_msg()
        human_array.header.frame_id = "camera_color_optical_frame"

        depth_img = self.latest_depth.copy()
        vis_img = rgb.copy()

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            if cls_id != self.person_cls or conf < self.conf_thresh:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx_bb = (x1 + x2) // 2
            cy_bb = (y1 + y2) // 2

            # Sample depth from center ROI of bounding box
            roi_w = max(1, int((x2 - x1) * self.depth_roi))
            roi_h = max(1, int((y2 - y1) * self.depth_roi))
            rx1 = max(0, cx_bb - roi_w // 2)
            rx2 = min(depth_img.shape[1], cx_bb + roi_w // 2)
            ry1 = max(0, cy_bb - roi_h // 2)
            ry2 = min(depth_img.shape[0], cy_bb + roi_h // 2)

            depth_roi_img = depth_img[ry1:ry2, rx1:rx2].astype(np.float32)
            depth_roi_img = depth_roi_img[depth_roi_img > 0]  # Remove zeros

            if depth_roi_img.size == 0:
                continue

            # Median depth in mm -> meters
            depth_m = float(np.median(depth_roi_img)) / 1000.0

            if depth_m <= 0 or depth_m > self.max_depth:
                continue

            # Back-project to 3D (camera frame)
            X = (cx_bb - self.cx) * depth_m / self.fx
            Y = (cy_bb - self.cy) * depth_m / self.fy
            Z = depth_m

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

            # Visualization
            color = (0, 255, 80)
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)
            label = f"person {conf:.2f} | {depth_m:.2f}m"
            cv2.putText(vis_img, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.circle(vis_img, (cx_bb, cy_bb), 4, (0, 255, 255), -1)

        self.humans_pub.publish(human_array)

        # Publish visualization
        vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
        vis_msg.header = human_array.header
        self.vis_pub.publish(vis_msg)


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
