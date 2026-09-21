#!/usr/bin/env python3
"""
Launch file: Chạy Model AI CNN trên Laptop (Offloading).
Chức năng:
  - Camera cắm trực tiếp vào Laptop (Intel RealSense D435 hoặc Webcam).
  - Laptop chạy mạng nơ-ron CNN ONNX bằng CPU/GPU mạnh mẽ của Laptop (30 - 60+ FPS thay vì 2 FPS trên Pi).
  - Hiển thị trực tiếp màn hình HUD góc lái & mask bám luống trên màn hình Laptop.
  - Chế độ mode:=server (Mặc định): Laptop tính toán AI rồi gửi góc lái (/crop_row/detection) về Pi qua Wi-Fi.
  - Chế độ mode:=driver: Laptop chạy toàn bộ khối điều khiển và gửi thẳng /cmd_vel về Pi qua Wi-Fi.
  - Pi không cần cắm camera, giải phóng 100% tài nguyên CPU và không bị lag Wi-Fi!

Sử dụng:
  ros2 launch my_robot_bringup laptop_cnn.launch.py
  ros2 launch my_robot_bringup laptop_cnn.launch.py view:=false (chạy ngầm không mở cửa sổ ảnh)
  ros2 launch my_robot_bringup laptop_cnn.launch.py mode:=driver (chạy toàn bộ driver trên Laptop)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        description='Khởi chạy camera_publisher trực tiếp trên Laptop để đọc từ cổng USB')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='server',
        description='Chế độ chạy trên Laptop: server (gửi góc lái về Pi) hoặc driver (gửi cmd_vel về Pi)')

    view_arg = DeclareLaunchArgument(
        'view', default_value='true',
        description='Mở cửa sổ HUD trực tiếp trên Laptop để quan sát Camera + Mask AI + Thước đo góc lái (view:=true)')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Mở RViz2 trực tiếp trên Laptop để quan sát mô hình xe và cảm biến (rviz:=true)')

    # ── Camera Publisher trên Laptop (Đọc trực tiếp từ cổng USB Laptop) ──
    camera_publisher_node = Node(
        package='my_robot_bringup',
        executable='camera_publisher',
        name='camera_publisher_laptop',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'camera_frame_id': 'camera_link',
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/local/image_raw'),
            ('/camera/image_raw', '/camera/local/image_raw_alt'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )

    # ── Chế độ 1 (Mặc định): CNN Server (Laptop suy luận AI ➔ Bắn góc lái về Pi) ──
    cnn_server_node = Node(
        package='my_robot_controller',
        executable='cnn_server',
        name='cnn_inference_server',
        output='screen',
        parameters=[{
            'input_height': 384,
            'input_width': 384,
            'roi_ratio': 0.80,
            'mask_threshold': 0.35,
            'max_steering_angle_deg': 14.0,
            'show_window': LaunchConfiguration('view'),
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/local/image_raw'),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'server'"]))
    )

    # ── Chế độ 2: CNN Driver (Laptop chạy toàn bộ FSM + SMC ➔ Bắn cmd_vel về Pi) ──
    cnn_driver_node = Node(
        package='my_robot_controller',
        executable='cnn_driver',
        name='cnn_driver_node',
        output='screen',
        parameters=[
            params_real,
            {
                'image_topic': '/camera/local/image_raw',
                'odom_topic': '/odometry/filtered',
                'imu_topic': '/imu/data',
            }
        ],
        remappings=[
            ('camera/image_raw', '/camera/color/image_raw'),
            ('odom', '/odometry/filtered'),
            ('/imu', '/imu/data'),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'driver'"]))
    )

    # ── RViz2 Visualization (Tùy chọn mở khi truyền rviz:=true) ──────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        camera_device_arg,
        enable_camera_arg,
        mode_arg,
        view_arg,
        rviz_arg,
        camera_publisher_node,
        cnn_server_node,
        cnn_driver_node,
        rviz_node,
    ])
