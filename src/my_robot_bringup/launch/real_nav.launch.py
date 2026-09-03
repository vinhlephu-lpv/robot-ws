"""
Launch file TỰ HÀNH THẬT với Nav2 trên Raspberry Pi.
Yêu cầu: Đã quét bản đồ trước đó bằng real_slam.launch.py.
Bao gồm:
  - Toàn bộ hệ thống xe thật (real_robot.launch.py)
  - Nav2 Stack (AMCL Localization + Planner + Controller + Collision Monitor)
  - Nạp bản đồ từ file .yaml

Sử dụng:
  ros2 launch my_robot_bringup real_nav.launch.py map:=/path/to/my_map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_real_params.yaml')

    # ── Launch Arguments ─────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/rplidar',
        description='RPLIDAR C1 serial port')

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Full path to map yaml file')

    # ── Include Real Robot Bringup (LiDAR + ESP32 + IMU + Madgwick + EKF) ─
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'enable_esp32': 'true',
            'enable_camera': 'false',
            'enable_cnn': 'false',
            'enable_rviz': 'false',
            'enable_imu': 'true',
            'enable_madgwick': 'true',
            'enable_ekf': 'true',
        }.items()
    )

    # ── Nav2 Localization (AMCL + Map Server) ────────────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
        }.items()
    )

    # ── Nav2 Navigation (Planner + Controller + Collision Monitor) ───
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_params,
        }.items()
    )

    return LaunchDescription([
        serial_port_arg,
        map_arg,
        real_robot_launch,
        localization_launch,
        navigation_launch,
    ])
