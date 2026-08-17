"""
Launch file QUÉT BẢN ĐỒ THẬT (SLAM Mapping) trên xe thật (Raspberry Pi / Laptop).
Bao gồm:
  - Toàn bộ hệ thống xe thật (Robot Model URDF, TF tree, RPLIDAR C1)
  - SLAM Toolbox (Scan Matching + Loop Closure)
  - Costmap Inflation Node (Bản đồ chi phí / Vùng an toàn thời gian thực)
  - RViz2 (Hiện xe 3D, tia quét LiDAR, Bản đồ SLAM, Costmap)

Sử dụng:
  ros2 launch my_robot_bringup real_slam.launch.py
  ros2 launch my_robot_bringup real_slam.launch.py serial_port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(pkg_bringup, 'config', 'slam_real_params.yaml')

    # ── Launch Arguments ─────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='RPLIDAR C1 serial port')

    enable_esp32_arg = DeclareLaunchArgument(
        'enable_esp32', default_value='true',
        description='Set true if ESP32 motor encoder bridge is connected')

    enable_costmap_arg = DeclareLaunchArgument(
        'enable_costmap', default_value='true',
        description='Enable real-time costmap inflation node')

    # ── Include Real Robot Bringup (URDF 3D model + TF + LiDAR C1 + RViz) ───
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'enable_esp32': LaunchConfiguration('enable_esp32'),
            'enable_camera': 'false',
            'enable_cnn': 'false',
            'enable_rviz': 'true',
        }.items()
    )

    # ── SLAM Toolbox (Standard Online Async Mapping) ─────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items()
    )

    # ── Real-time Costmap Inflation Node ─────────────────────────────
    costmap_node = Node(
        package='my_robot_navigation',
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('enable_costmap'))
    )

    return LaunchDescription([
        serial_port_arg,
        enable_esp32_arg,
        enable_costmap_arg,
        real_robot_launch,
        slam_launch,
        costmap_node,
    ])
