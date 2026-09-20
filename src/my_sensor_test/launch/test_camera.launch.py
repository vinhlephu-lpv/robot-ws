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
        default_value='realsense',
        description='Camera input: realsense (Intel D435), http://... (iPhone), /dev/video* (Webcam)'
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

    # ── Camera Publisher Node (Hỗ trợ RealSense D435, iPhone & Webcam) ──
    camera_node = Node(
        package='my_robot_bringup',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'width': LaunchConfiguration('image_width'),
            'height': LaunchConfiguration('image_height'),
            'fps': 30.0,
            'camera_frame_id': 'camera_link',
        }],
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
        camera_node,
        rqt_image_view_node,
    ])
