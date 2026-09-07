"""
Launch file TỰ HÀNH THẬT với Nav2 trên Raspberry Pi (KHÔNG CẦN BẢN ĐỒ).
Chế độ Odom-Only: Robot điều hướng dựa trên EKF odometry + LiDAR obstacle avoidance.
Người dùng click "2D Nav Goal" trên RViz (PC) để chọn điểm đích.

Pipeline:
  - Pi: LiDAR C1 + ESP32 Encoder + IMU + Madgwick + EKF → /odometry/filtered
  - Pi: Nav2 (SmacPlanner2D + RegulatedPurePursuit + Costmaps + CollisionMonitor)
  - PC: RViz2 (Fixed Frame = odom) → Click "2D Nav Goal" → /navigate_to_pose action

Sử dụng:
  ros2 launch my_robot_bringup real_nav.launch.py
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

    # ── Nav2 Navigation (Planner + Controller + Costmaps + CollisionMonitor) ─
    # Chế độ Odom-Only: global_frame = odom, rolling_window = true
    # Không cần AMCL / Map Server vì chạy ngoài trời không có bản đồ
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
        real_robot_launch,
        navigation_launch,
    ])

