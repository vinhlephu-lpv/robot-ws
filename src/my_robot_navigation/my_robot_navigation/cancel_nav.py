#!/usr/bin/env python3
"""
Emergency Navigation Goal Cancellation Tool for Nav2
Cancels active Nav2 navigation goals and commands the robot to immediate stop.
Usage:
  ros2 run my_robot_navigation cancel_nav
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses


class CancelNavNode(Node):
    def __init__(self):
        super().__init__('cancel_nav_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.get_logger().info("Đang gửi lệnh hủy mục tiêu điều hướng (Nav2 Cancel)...")

    def cancel_all(self):
        # Publish 0 velocity to immediately halt physical movement
        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)

        # Cancel NavigateToPose if active
        if self.nav_to_pose_client.server_is_ready():
            self.get_logger().info("Đã kết nối với /navigate_to_pose, đang yêu cầu hủy goal...")
            future = self.nav_to_pose_client._cancel_goal_async(None)

        # Cancel NavigateThroughPoses if active
        if self.nav_through_poses_client.server_is_ready():
            self.get_logger().info("Đã kết nối với /navigate_through_poses, đang yêu cầu hủy goal...")
            future2 = self.nav_through_poses_client._cancel_goal_async(None)

        # Final stop command
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)

        self.get_logger().info(" Đã hủy thành công mục tiêu Nav2! Xe đã dừng lại hoàn toàn. Bạn có thể vẽ lại Goal mới (bấm phím G).")


def main(args=None):
    rclpy.init(args=args)
    node = CancelNavNode()
    try:
        node.cancel_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
