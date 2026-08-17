"""
Launch file cho XE THẬT (Raspberry Pi + Camera + BTS7960).
KHÔNG dùng Gazebo. Chỉ cần camera thật và motor driver.

Yêu cầu trên Raspberry Pi:
  sudo apt install ros-jazzy-v4l2-camera
  pip3 install RPi.GPIO

Sử dụng:
  ros2 launch my_robot_controller real_robot.launch.py

Kiểm tra camera trước:
  ls /dev/video*
  ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
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
        description='V4L2 camera device path'
    )
    camera_device = LaunchConfiguration('camera_device')

    # ── Camera Node (v4l2_camera) ─────────────────────────────────────
    # Publish /camera/image_raw — cùng topic với simulation
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_size':   [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'YUYV',
        }],
        remappings=[
            ('image_raw', 'camera/image_raw'),
        ]
    )

    # ── BTS7960 Motor Driver ──────────────────────────────────────────
    # Subscribe /cmd_vel → điều khiển GPIO
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
        camera_node,
        motor_driver,
        cnn_driver,
    ])
