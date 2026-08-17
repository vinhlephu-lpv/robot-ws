#!/usr/bin/env python3
"""
Launch file to test real LiDAR RPLIDAR C1 (C1M1) + SLAM Mapping in RViz2.
Uses official sllidar_ros2 driver for C1 series with ROS 2 Jazzy TF format.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_sensor_test')
    slam_params = os.path.join(pkg_share, 'config', 'slam_real_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'test_slam.rviz')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR USB serial port'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='LiDAR baudrate (C1: 460800)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='LiDAR frame ID'
    )

    # Static TF: odom -> base_link (ROS 2 Jazzy format)
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'odom', '--child-frame-id', 'base_link']
    )

    # Static TF: base_link -> laser (Rotated 180 deg for correct front heading)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.2', '--yaw', '3.14159265', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser']
    )

    # Official SLLIDAR C1 Driver Node
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    # SLAM Toolbox Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': False}
        ]
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
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        static_tf_odom,
        static_tf_laser,
        lidar_node,
        slam_node,
        rviz2_node,
    ])
