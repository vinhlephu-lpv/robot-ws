"""
Launch file khởi động TOÀN BỘ HỆ THỐNG XE THẬT (Raspberry Pi).
Bao gồm:
  - Robot State Publisher (URDF 3D model + TF tree)
  - Joint State Publisher
  - RPLIDAR C1 (sllidar_ros2, laser_frame, 460800 baud)
  - Astra Mini S Camera (remap -> /camera/image_raw)
  - BTS7960 Motor Driver (GPIO)
  - CNN Driver (Tự hành theo luống bắp)

Sử dụng:
  ros2 launch my_robot_bringup real_robot.launch.py
  ros2 launch my_robot_bringup real_robot.launch.py serial_port:=/dev/rplidar
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_ctrl = get_package_share_directory('my_robot_controller')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'real_robot.rviz')
    params_real = os.path.join(pkg_ctrl, 'config', 'params_real.yaml')

    robot_description = Command(['xacro ', '"', xacro_file, '"'])

    # ── Launch Arguments ─────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='RPLIDAR C1 serial port')

    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Astra Mini S / USB camera device')

    # ── Robot State Publisher (URDF + TF) ────────────────────────────
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

    # ── RPLIDAR C1 (Official sllidar_ros2) ───────────────────────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    # ── Astra Mini S Camera ──────────────────────────────────────────
    # Remap /camera/color/image_raw -> /camera/image_raw
    # to match simulation topic used by CNN Driver
    camera_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        name='astra_camera',
        output='screen',
        parameters=[{
            'color_width': 1280,
            'color_height': 720,
            'color_fps': 30,
            'enable_color': True,
            'enable_depth': True,
            'enable_pointcloud': False,
            'camera_link_frame_id': 'camera_link',
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/image_raw'),
        ]
    )

    # ── ESP32 Hardware Bridge (Motor PID + Encoder Odom) ─────────────
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port', default_value='/dev/ttyUSB1',
        description='ESP32 serial port')

    esp32_bridge = Node(
        package='my_robot_bringup',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen',
        parameters=[{
            'connection_mode': 'serial',
            'serial_port': LaunchConfiguration('esp32_port'),
            'baudrate': 115200,
            'wheel_diameter': 0.20,
            'wheel_base': 0.58,
            'publish_tf': True,
        }]
    )

    # ── CNN Driver (Autonomous Row Following) ────────────────────────
    cnn_driver = Node(
        package='my_robot_controller',
        executable='cnn_driver',
        name='cnn_driver_node',
        output='screen',
        parameters=[params_real]
    )

    # ── RViz2 ────────────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        serial_port_arg,
        camera_device_arg,
        esp32_port_arg,
        robot_state_pub,
        joint_state_pub,
        lidar_node,
        camera_node,
        esp32_bridge,
        cnn_driver,
        rviz2_node,
    ])
