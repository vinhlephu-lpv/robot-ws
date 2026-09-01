#!/usr/bin/env python3
"""
Launch file to test Camera (Orbbec Astra Pro / Series 3D or USB Webcam) independently.
Starts camera driver node and rqt_image_view GUI.

Usage:
  ros2 launch my_sensor_test test_camera.launch.py                       # Mặc định: Orbbec Astra 3D
  ros2 launch my_sensor_test test_camera.launch.py camera_driver:=v4l2   # Revert: USB Webcam
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    camera_driver_arg = DeclareLaunchArgument(
        'camera_driver',
        default_value='astra',
        description='Camera driver type: astra (Orbbec Astra Pro/3D) or v4l2 (USB Webcam)'
    )

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device path for webcam (e.g. /dev/video0, /dev/video2)'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Camera image width'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Camera image height'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable Astra Depth stream'
    )

    enable_point_cloud_arg = DeclareLaunchArgument(
        'enable_point_cloud',
        default_value='true',
        description='Enable Astra 3D Point Cloud stream'
    )

    # ── V4L2 USB Webcam Node ──────────────────────────────────────────
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='camera',
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

    # ── Orbbec Astra Camera Node (OpenNI2 + UVC) ─────────────────────
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
            'color_width': LaunchConfiguration('image_width'),
            'color_height': LaunchConfiguration('image_height'),
            'color_fps': 30,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'enable_color': True,
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_ir': False,
            'enable_point_cloud': LaunchConfiguration('enable_point_cloud'),
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

    # ── GUI rqt_image_view ────────────────────────────────────────────
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        arguments=['/camera/color/image_raw']
    )

    return LaunchDescription([
        camera_driver_arg,
        video_device_arg,
        image_width_arg,
        image_height_arg,
        enable_depth_arg,
        enable_point_cloud_arg,
        v4l2_camera_node,
        astra_camera_node,
        rqt_image_view_node,
    ])
