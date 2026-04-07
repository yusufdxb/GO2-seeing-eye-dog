#!/usr/bin/env python3
"""Set GO2 joints to stand pose via Gazebo service, then exit."""

import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelConfiguration


JOINT_NAMES = [
    'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
    'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
    'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
    'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
]

STAND_POSITIONS = [
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
]


def main():
    rclpy.init()
    node = Node('set_stand_pose')

    client = node.create_client(
        SetModelConfiguration, '/gazebo/set_model_configuration'
    )

    node.get_logger().info('Waiting for /gazebo/set_model_configuration...')
    if not client.wait_for_service(timeout_sec=30.0):
        node.get_logger().error('Service not available after 30s')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    req = SetModelConfiguration.Request()
    req.model_name = 'go2'
    req.urdf_param_name = 'robot_description'
    req.joint_names = JOINT_NAMES
    req.joint_positions = STAND_POSITIONS

    node.get_logger().info('Setting joints to stand pose...')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None:
        result = future.result()
        if result.success:
            node.get_logger().info(f'Success: {result.status_message}')
        else:
            node.get_logger().error(f'Failed: {result.status_message}')
    else:
        node.get_logger().error('Service call timed out')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
