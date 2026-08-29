#!/usr/bin/env python3
"""
Launch file to test real ICM-20948 IMU sensor on Raspberry Pi (I2C).
Publishes /imu topic with Accel, Gyro, and filtered Orientation Quaternion.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', '"', xacro_file, '"'])

    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus', default_value='1',
        description='I2C bus number (/dev/i2c-1 on Raspberry Pi)')

    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address', default_value='104', # 0x68 = 104 decimal
        description='ICM-20948 I2C Address (0x68=104, 0x69=105)')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    imu_driver_node = Node(
        package='my_robot_controller',
        executable='imu_driver',
        name='imu_driver_node',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': LaunchConfiguration('i2c_address'),
            'frame_id': 'imu_link',
            'publish_topic': '/imu',
            'rate_hz': 50.0,
            'calibrate_samples': 60,
        }]
    )

    return LaunchDescription([
        i2c_bus_arg,
        i2c_address_arg,
        robot_state_pub,
        joint_state_pub,
        imu_driver_node,
    ])
