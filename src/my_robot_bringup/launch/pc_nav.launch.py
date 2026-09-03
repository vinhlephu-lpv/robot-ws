"""
Launch file chạy Nav2 Navigation Stack trên PC kết nối với SLAM thời gian thực trên Pi.
Dùng cho mô hình:
  - Pi: chạy real-slam (phần cứng + SLAM Toolbox tạo /map và TF map -> odom)
  - PC: chạy RViz và pc_nav (A* Global Planner + Regulated Pure Pursuit + Global/Local Costmaps)
Không chứa AMCL và không bật lại phần cứng để tránh xung đột cổng Serial và xung đột TF.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_real_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to the ROS2 parameters file to use'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ── Nav2 Navigation Only (Planner + Controller + Costmaps) ───────
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        navigation_launch,
    ])
