"""
Launch file cho XE THẬT (Raspberry Pi / Laptop + USB Webcam + BTS7960 + CNN).
KHÔNG dùng Gazebo. Chỉ cần camera thật và motor driver.

Sử dụng:
  ros2 launch my_robot_controller real_robot.launch.py
  ros2 launch my_robot_controller real_robot.launch.py camera_device:=/dev/video0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ctrl = get_package_share_directory('my_robot_controller')
    params_file = os.path.join(pkg_ctrl, 'config', 'params_real.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='V4L2 camera device path (/dev/video0)'
    )
    camera_device = LaunchConfiguration('camera_device')

    # ── Camera Node (v4l2_camera) ─────────────────────────────────────
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_size':   [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'MJPG',
        }],
        remappings=[
            ('image_raw', 'camera/image_raw'),
        ],
    )

    # ── BTS7960 Motor Driver ──────────────────────────────────────────
    motor_driver = Node(
        package='my_robot_controller',
        executable='bts7960_driver',
        name='bts7960_driver_node',
        output='screen',
        parameters=[params_file]
    )

    # ── CNN Driver ────────────────────────────────────────────────────
    cnn_driver = Node(
        package='my_robot_controller',
        executable='cnn_driver',
        name='cnn_driver_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        camera_device_arg,
        v4l2_camera_node,
        motor_driver,
        cnn_driver,
    ])
