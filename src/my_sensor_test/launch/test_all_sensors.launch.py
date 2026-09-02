#!/usr/bin/env python3
"""
Launch file to test BOTH real USB Webcam and LiDAR simultaneously.
Publishes static TF between laser and camera, and displays both in RViz2.

Usage:
  ros2 launch my_sensor_test test_all_sensors.launch.py
  ros2 launch my_sensor_test test_all_sensors.launch.py video_device:=/dev/video0 serial_port:=/dev/rplidar
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_sensor_test')
    rviz_config = os.path.join(pkg_share, 'rviz', 'test_sensors.rviz')

    # Camera args
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device path (/dev/video0)'
    )

    # LiDAR args
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='LiDAR USB serial port (/dev/rplidar, /dev/ttyUSB0)'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='LiDAR baudrate (460800 for C1, 115200 for A1)'
    )

    # ── V4L2 USB Webcam Node ──────────────────────────────────────────
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'MJPG',
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ],
    )

    # ── RPLIDAR C1 / A1 Node ──────────────────────────────────────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    # ── Static TF Publisher: laser -> camera_link ────────────────────
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_camera_tf',
        arguments=['0.15', '0.0', '-0.05', '0.0', '0.0', '0.0', 'laser', 'camera_link']
    )

    # ── RViz2 Node ───────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        video_device_arg,
        serial_port_arg,
        serial_baudrate_arg,
        v4l2_camera_node,
        lidar_node,
        static_tf_node,
        rviz2_node,
    ])
