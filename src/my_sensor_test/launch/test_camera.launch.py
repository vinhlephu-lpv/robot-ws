#!/usr/bin/env python3
"""
Launch file to test USB Webcam independently.
Starts v4l2_camera driver node and rqt_image_view GUI.

Usage:
  ros2 launch my_sensor_test test_camera.launch.py
  ros2 launch my_sensor_test test_camera.launch.py video_device:=/dev/video0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device path for webcam (e.g. /dev/video0, /dev/video2)'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Camera image width'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Camera image height'
    )

    # ── V4L2 USB Webcam Node ──────────────────────────────────────────
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='camera',
        name='camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [LaunchConfiguration('image_width'), LaunchConfiguration('image_height')],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'MJPG',
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ],
    )

    # ── GUI rqt_image_view ────────────────────────────────────────────
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        arguments=['/camera/color/image_raw']
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        v4l2_camera_node,
        rqt_image_view_node,
    ])
