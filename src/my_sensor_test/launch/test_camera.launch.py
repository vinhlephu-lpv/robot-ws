#!/usr/bin/env python3
"""
Launch file to test USB Camera / Webcam independently.
Starts v4l2_camera node and rqt_image_view GUI.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device path (e.g. /dev/video0, /dev/video1)'
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

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'YUYV',
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
        ]
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        arguments=['/camera/image_raw']
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        camera_node,
        rqt_image_view_node,
    ])
