"""
Launch file QUÉT BẢN ĐỒ THẬT (SLAM Mapping) trên Raspberry Pi.
Bao gồm:
  - Toàn bộ hệ thống xe thật (real_robot.launch.py)
  - SLAM Toolbox (scan matching + loop closing)
  - RViz2 (bản đồ + tia quét)

Sử dụng:
  ros2 launch my_robot_bringup real_slam.launch.py
  # Cầm xe đi quanh phòng/vườn bắp để vẽ bản đồ.
  # Lưu bản đồ: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
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

    slam_params = os.path.join(pkg_bringup, 'config', 'slam_real_params.yaml')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'real_robot.rviz')

    # ── Launch Arguments ─────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='RPLIDAR C1 serial port')

    # ── Include Real Robot Bringup (sensors + controllers) ───────────
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
        }.items()
    )

    # ── SLAM Toolbox (Real Hardware Mode) ────────────────────────────
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

    return LaunchDescription([
        serial_port_arg,
        real_robot_launch,
        slam_node,
    ])
