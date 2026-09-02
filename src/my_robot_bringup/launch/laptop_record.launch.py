"""
Launch file: Thu thập dữ liệu Dataset trực tiếp qua RViz (Chạy trên Laptop đặt trên xe).

Bao gồm:
  - Robot State Publisher + Joint State Publisher + Static Odom TF: Hiển thị mô hình 3D xe trên RViz không lỗi TF.
  - USB Webcam V4L2 Driver.
  - Camera Recorder Node: Đọc camera USB, vừa ghi video MP4 vừa tách ảnh frame vào dataset.
  - RViz2: Hiển thị xe 3D và khung camera thời gian thực.

Sử dụng:
  ros2 launch my_robot_bringup laptop_record.launch.py
  ros2 launch my_robot_bringup laptop_record.launch.py name:=luong_1
"""

import os
import subprocess
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_desc = get_package_share_directory('my_robot_description')

    rviz_config = os.path.join(pkg_bringup, 'rviz', 'record_rviz.rviz')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── Arguments ──────────────────────────────────────────────────────────
    name_arg = DeclareLaunchArgument(
        'name', default_value='',
        description='Tên phiên quay (ví dụ: luong_1, luong_2)')

    device_arg = DeclareLaunchArgument(
        'device', default_value='auto',
        description='Cổng camera USB (/dev/video0, auto)')

    width_arg = DeclareLaunchArgument(
        'width', default_value='640',
        description='Chiều rộng khung hình')

    height_arg = DeclareLaunchArgument(
        'height', default_value='480',
        description='Chiều cao khung hình')

    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30.0',
        description='FPS ghi video (30.0 hoặc 60.0)')

    interval_arg = DeclareLaunchArgument(
        'interval', default_value='0.333',
        description='Khoảng cách thời gian tách frame (giây) vào thư mục imgs (0.333s = 3 ảnh/giây)')

    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', default_value='true',
        description='Mở giao diện RViz2')

    # ── Robot Model Nodes ──────────────────────────────────────────────────
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

    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'odom', '--child-frame-id', 'base_footprint']
    )

    # ── Camera & Recorder Node ────────────────────────────────────────────
    camera_recorder_node = Node(
        package='my_robot_bringup',
        executable='camera_recorder',
        name='camera_recorder',
        output='screen',
        parameters=[{
            'mode': 'auto',
            'device': LaunchConfiguration('device'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'record_name': LaunchConfiguration('name'),
            'extract_interval': LaunchConfiguration('interval'),
            'topic': '/camera/color/image_raw',
        }]
    )

    # ── RViz2 ──────────────────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    return LaunchDescription([
        name_arg,
        device_arg,
        width_arg,
        height_arg,
        fps_arg,
        interval_arg,
        open_rviz_arg,
        static_odom_tf,
        robot_state_pub,
        joint_state_pub,
        camera_recorder_node,
        rviz2_node,
    ])
