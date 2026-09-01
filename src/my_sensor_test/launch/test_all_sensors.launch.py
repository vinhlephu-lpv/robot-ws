#!/usr/bin/env python3
"""
Launch file to test BOTH real Camera (Orbbec Astra 3D or USB Webcam) and LiDAR simultaneously.
Publishes static TF between laser and camera, and displays both in RViz2.

Usage:
  ros2 launch my_sensor_test test_all_sensors.launch.py
  ros2 launch my_sensor_test test_all_sensors.launch.py camera_driver:=v4l2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_sensor_test')
    rviz_config = os.path.join(pkg_share, 'rviz', 'test_sensors.rviz')

    # Camera args
    camera_driver_arg = DeclareLaunchArgument(
        'camera_driver',
        default_value='astra',
        description='Camera driver type: astra (Orbbec Astra Pro/3D) or v4l2 (USB Webcam)'
    )

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device path'
    )

    # LiDAR args
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='LiDAR USB serial port (/dev/rplidar, /dev/ttyUSB0)'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='LiDAR baudrate (460800 for C1, 115200 for A1)'
    )

    # ── V4L2 USB Webcam Node ──────────────────────────────────────────
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'MJPG',
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_driver'), "' in ['v4l2', 'webcam']"])
        )
    )

    # ── Orbbec Astra 3D Camera Node (OpenNI2 + UVC) ───────────────────
    camera_env = dict(os.environ)
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_astra_prefix = get_package_prefix('astra_camera')
        openni2_drivers_dir = os.path.join(pkg_astra_prefix, 'lib', 'OpenNI2', 'Drivers')
        openni2_lib_dir = os.path.join(pkg_astra_prefix, 'lib')
        if os.path.exists(openni2_drivers_dir):
            camera_env['OPENNI2_REDIST'] = openni2_drivers_dir
            camera_env['OPENNI2_DRIVERS_PATH'] = openni2_drivers_dir
        if os.path.exists(openni2_lib_dir):
            camera_env['LD_LIBRARY_PATH'] = f"{openni2_lib_dir}:{camera_env.get('LD_LIBRARY_PATH', '')}"
    except Exception:
        pass

    astra_camera_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        env=camera_env,
        parameters=[{
            'camera_name': 'camera',
            'vendor_id': 0,
            'product_id': 0,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'enable_color': True,
            'enable_depth': True,
            'enable_ir': False,
            'enable_point_cloud': True,
            'enable_colored_point_cloud': False,
            'use_uvc_camera': True,
            'uvc_vendor_id': 0x2bc5,
            'uvc_product_id': 0x0501,
            'uvc_retry_count': 100,
            'uvc_camera_format': 'mjpeg',
            'oni_log_level': 'none',
            'oni_log_to_console': False,
            'oni_log_to_file': False,
            'publish_tf': True,
            'tf_publish_rate': 10.0,
            'camera_link_frame_id': 'camera_link',
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_driver'), "' == 'astra'"])
        )
    )

    # ── RPLIDAR C1 / A1 Node ──────────────────────────────────────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    # ── Static TF Publisher: laser -> camera_link ────────────────────
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_camera_tf',
        arguments=['0.15', '0.0', '-0.05', '0.0', '0.0', '0.0', 'laser', 'camera_link']
    )

    # ── RViz2 Node ───────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        camera_driver_arg,
        video_device_arg,
        serial_port_arg,
        serial_baudrate_arg,
        v4l2_camera_node,
        astra_camera_node,
        lidar_node,
        static_tf_node,
        rviz2_node,
    ])
