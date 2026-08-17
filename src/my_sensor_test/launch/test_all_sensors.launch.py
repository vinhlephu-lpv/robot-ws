#!/usr/bin/env python3
"""
Launch file to test BOTH real Camera and LiDAR simultaneously.
Publishes static TF between laser and camera, and displays both in RViz2.
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
        description='V4L2 camera device path'
    )

    # LiDAR args
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR USB serial port'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='LiDAR baudrate (115200 or 256000)'
    )

    # Camera Node
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

    # LiDAR Node
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
        }]
    )

    # Static TF Publisher: laser -> camera_link (camera positioned 0.15m in front of lidar)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_camera_tf',
        arguments=['0.15', '0.0', '-0.05', '0.0', '0.0', '0.0', 'laser', 'camera_link']
    )

    # RViz2 Node
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
        camera_node,
        lidar_node,
        static_tf_node,
        rviz2_node,
    ])
