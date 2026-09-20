#!/usr/bin/env python3
"""
Launch file: Chạy Model AI CNN trên Laptop (Offloading).
Chức năng:
  - Laptop nhận luồng hình ảnh trực tiếp từ điện thoại (iPhone/Android qua cáp sạc USB).
  - Laptop chạy mạng nơ-ron CNN ONNX bằng CPU/GPU mạnh mẽ của Laptop (30 - 60+ FPS thay vì 2 FPS trên Pi).
  - Laptop đăng ký nhận /odometry/filtered, /imu/data, /scan từ Raspberry Pi qua Wi-Fi.
  - Laptop tính toán bẻ lái SMC + tránh vật cản + quay đầu và phát /cmd_vel điều khiển xe thật trên Pi.
  - Tùy chọn mở RViz2 quan sát (view:=true).

Sử dụng:
  ros2 launch my_robot_bringup laptop_cnn.launch.py
  ros2 launch my_robot_bringup laptop_cnn.launch.py view:=true
  ros2 launch my_robot_bringup laptop_cnn.launch.py camera_device:=http://172.20.10.1:4747/video
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_ctrl = get_package_share_directory('my_robot_controller')

    rviz_config = os.path.join(pkg_desc, 'rviz', 'display.rviz')
    params_real = os.path.join(pkg_ctrl, 'config', 'params_real.yaml')

    # ── Launch Arguments ─────────────────────────────────────────────
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='realsense',
        description='Nguồn camera: realsense (Intel D435), http://172.20.10.1:4747/video (iPhone), /dev/video* (Webcam)')

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='Khởi chạy camera_publisher trực tiếp trên Laptop để đọc từ cáp sạc')

    view_arg = DeclareLaunchArgument(
        'view', default_value='false',
        description='Mở RViz2 trực tiếp trên Laptop để quan sát luồng xe và camera (view:=true)')

    # ── Camera Publisher trên Laptop (Đọc trực tiếp từ cáp sạc USB) ──
    camera_publisher_node = Node(
        package='my_robot_bringup',
        executable='camera_publisher',
        name='camera_publisher_laptop',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'width': 640,
            'height': 480,
            'fps': 15.0,
            'camera_frame_id': 'camera_link',
        }],
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )

    # ── CNN Driver Node (Chạy trên Laptop với FPS cao vượt trội) ─────
    cnn_driver_node = Node(
        package='my_robot_controller',
        executable='cnn_driver',
        name='cnn_driver_node',
        output='screen',
        parameters=[
            params_real,
            {
                'image_topic': '/camera/color/image_raw',
                'odom_topic': '/odometry/filtered',
                'imu_topic': '/imu/data',
            }
        ],
        remappings=[
            ('camera/image_raw', '/camera/color/image_raw'),
            ('odom', '/odometry/filtered'),
            ('/imu', '/imu/data'),
        ]
    )

    # ── RViz2 Visualization (Tùy chọn mở khi truyền view:=true) ──────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('view'))
    )

    return LaunchDescription([
        camera_device_arg,
        enable_camera_arg,
        view_arg,
        camera_publisher_node,
        cnn_driver_node,
        rviz_node,
    ])
