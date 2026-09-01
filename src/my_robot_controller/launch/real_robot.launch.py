"""
Launch file cho XE THẬT (Raspberry Pi / Laptop + Camera Astra/Webcam + BTS7960 + CNN).
KHÔNG dùng Gazebo. Chỉ cần camera thật và motor driver.

Sử dụng:
  ros2 launch my_robot_controller real_robot.launch.py                       # Mặc định: Orbbec Astra 3D Camera
  ros2 launch my_robot_controller real_robot.launch.py camera_driver:=v4l2   # Revert: USB Webcam
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ctrl = get_package_share_directory('my_robot_controller')
    params_file = os.path.join(pkg_ctrl, 'config', 'params_real.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    camera_driver_arg = DeclareLaunchArgument(
        'camera_driver',
        default_value='astra',
        description='Camera driver: astra (Orbbec Astra 3D) or v4l2 (USB Webcam)'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='V4L2 camera device path'
    )
    camera_device = LaunchConfiguration('camera_device')

    # ── Camera Node (v4l2_camera) ─────────────────────────────────────
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_size':   [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'MJPG',
        }],
        remappings=[
            ('image_raw', 'camera/image_raw'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_driver'), "' in ['v4l2', 'webcam']"])
        )
    )

    # ── Camera Node (Orbbec Astra 3D Camera) ───────────────────────────
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
            'enable_point_cloud': False,
            'use_uvc_camera': True,
            'uvc_vendor_id': 0x2bc5,
            'uvc_product_id': 0x0501,
            'uvc_retry_count': 100,
            'uvc_camera_format': 'mjpeg',
            'oni_log_level': 'none',
            'oni_log_to_console': False,
            'oni_log_to_file': False,
            'publish_tf': True,
            'camera_link_frame_id': 'camera_link',
        }],
        remappings=[
            ('/camera/color/image_raw', 'camera/image_raw'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_driver'), "' == 'astra'"])
        )
    )

    # ── BTS7960 Motor Driver ──────────────────────────────────────────
    motor_driver = Node(
        package='my_robot_controller',
        executable='bts7960_driver',
        name='bts7960_driver_node',
        output='screen',
        parameters=[params_file]
    )

    # ── CNN Driver ────────────────────────────────────────────────────
    cnn_driver = Node(
        package='my_robot_controller',
        executable='cnn_driver',
        name='cnn_driver_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        camera_driver_arg,
        camera_device_arg,
        v4l2_camera_node,
        astra_camera_node,
        motor_driver,
        cnn_driver,
    ])
