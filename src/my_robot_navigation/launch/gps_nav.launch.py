#!/usr/bin/env python3
"""
==============================================================================
  Launch File: gps_nav.launch.py (Package: my_robot_navigation)
  
  Mục đích:
  TỰ HÀNH NGOÀI TRỜI THEO TỌA ĐỘ VỆ TINH GPS KẾT HỢP NAV2 CHO ROBOT THẬT.
  
  Nguyên lý:
  - EKF 2 (Global) đã phát TF map -> odom từ dữ liệu vệ tinh GPS NEO-M10.
  - EKF 1 (Local) đã phát TF odom -> base_footprint từ bánh xe và IMU Madgwick.
  - Không cần AMCL hay quét bản đồ trước (vì định vị hoàn toàn qua GPS ngoài trời).
  - Nav2 Controller & Costmap tự động né tránh vật cản bằng RPLIDAR C1 trong
    quá trình bám tọa độ GPS.
  - Node gps_waypoint_follower tự động chuyển Kinh/Vĩ độ sang Map và gửi Goal.
==============================================================================
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

    # ── Arguments ─────────────────────────────────────────────────────────
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints', default_value='',
        description='Đường dẫn tới file YAML danh sách tọa độ GPS waypoints'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Sử dụng thời gian mô phỏng'
    )

    # ── 1. Nav2 Navigation Stack (Không cần AMCL vì đã có EKF 2 Global) ───
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
    )

    # ── 2. GPS Waypoint Follower Node (Gọi Service /fromLL và gửi Goal) ────
    gps_follower_node = Node(
        package='my_robot_navigation',
        executable='gps_waypoint_follower',
        name='gps_waypoint_follower',
        output='screen',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints'),
            'map_frame': 'map',
            'goal_tolerance_m': 0.8,
        }]
    )

    return LaunchDescription([
        waypoints_file_arg,
        use_sim_time_arg,
        nav2_launch,
        gps_follower_node,
    ])
