#!/usr/bin/env python3
"""
Pipeline Metrics Logger Node
Subscribes to /go2/perception/metrics and /go2/safety_monitor/metrics,
and appends each message as a CSV row to a configurable output file.
A header row is written on the first message received.
"""

import csv
import os

import rclpy
from rclpy.node import Node

from go2_msgs.msg import PipelineMetrics

# Ordered field names that match PipelineMetrics exactly (downstream tooling depends on order)
_CSV_FIELDS = [
    "ros_stamp_sec",
    "ros_stamp_nanosec",
    "node_name",
    "frame_id",
    "cv_bridge_rgb_ms",
    "yolo_inference_ms",
    "depth_sampling_ms",
    "backproject_ms",
    "visualization_ms",
    "publish_ms",
    "safety_obstacle_ms",
    "safety_stair_ms",
    "safety_drop_ms",
    "safety_passage_ms",
    "safety_total_ms",
    "total_ms",
]


class PipelineMetricsLogger(Node):
    def __init__(self):
        super().__init__("pipeline_metrics_logger")

        self.declare_parameter("output_path", "/tmp/pipeline_metrics.csv")

        output_path: str = self.get_parameter("output_path").value

        # Open file; create parent dirs if needed
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        self._file = open(output_path, "w", newline="", buffering=1)  # line-buffered
        self._writer = csv.DictWriter(self._file, fieldnames=_CSV_FIELDS)
        self._header_written = False

        self.perception_sub = self.create_subscription(
            PipelineMetrics,
            "/go2/perception/metrics",
            self._metrics_callback,
            10,
        )
        self.safety_sub = self.create_subscription(
            PipelineMetrics,
            "/go2/safety_monitor/metrics",
            self._metrics_callback,
            10,
        )

        self.get_logger().info(
            f"PipelineMetricsLogger ready. Writing to {output_path}"
        )

    def _metrics_callback(self, msg: PipelineMetrics) -> None:
        if not self._header_written:
            self._writer.writeheader()
            self._header_written = True

        row = {
            "ros_stamp_sec": msg.header.stamp.sec,
            "ros_stamp_nanosec": msg.header.stamp.nanosec,
            "node_name": msg.node_name,
            "frame_id": msg.frame_id,
            "cv_bridge_rgb_ms": msg.cv_bridge_rgb_ms,
            "yolo_inference_ms": msg.yolo_inference_ms,
            "depth_sampling_ms": msg.depth_sampling_ms,
            "backproject_ms": msg.backproject_ms,
            "visualization_ms": msg.visualization_ms,
            "publish_ms": msg.publish_ms,
            "safety_obstacle_ms": msg.safety_obstacle_ms,
            "safety_stair_ms": msg.safety_stair_ms,
            "safety_drop_ms": msg.safety_drop_ms,
            "safety_passage_ms": msg.safety_passage_ms,
            "safety_total_ms": msg.safety_total_ms,
            "total_ms": msg.total_ms,
        }
        self._writer.writerow(row)

    def destroy_node(self):
        self._file.flush()
        self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PipelineMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
