"""
Launch file khởi động TOÀN BỘ HỆ THỐNG XE THẬT (Raspberry Pi / Laptop).
Bao gồm:
  - Robot State Publisher (URDF 3D model + TF tree)
  - Joint State Publisher
  - RPLIDAR C1 (sllidar_ros2, laser_frame, 460800 baud)
  - Static Odom TF (hoặc ESP32 Hardware Bridge nếu enable_esp32:=true)
  - Astra Mini S Camera (tùy chọn enable_camera:=true)
  - CNN Driver (tùy chọn enable_cnn:=true)
  - RViz2 (tùy chọn enable_rviz:=true)

Sử dụng:
  ros2 launch my_robot_bringup real_robot.launch.py
  ros2 launch my_robot_bringup real_robot.launch.py serial_port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
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
        'serial_port', default_value='/dev/rplidar',
        description='RPLIDAR C1 serial port')

    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Astra Mini S / USB camera device')

    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port', default_value='/dev/esp32',
        description='ESP32 serial port')

    enable_esp32_arg = DeclareLaunchArgument(
        'enable_esp32', default_value='true',
        description='Enable ESP32 motor/odom bridge (if false, publishes static odom TF)')

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='Enable Astra camera node')

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth', default_value='false',
        description='Enable depth stream (false saves USB bandwidth and maximizes color 30 FPS)')

    color_width_arg = DeclareLaunchArgument(
        'color_width', default_value='640',
        description='Color image width (640 for VGA)')

    color_height_arg = DeclareLaunchArgument(
        'color_height', default_value='480',
        description='Color image height (480 for VGA)')

    enable_cnn_arg = DeclareLaunchArgument(
        'enable_cnn', default_value='false',
        description='Enable CNN row-following driver')

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz', default_value='false',
        description='Enable RViz2 visualization')

    camera_driver_arg = DeclareLaunchArgument(
        'camera_driver', default_value='v4l2',
        description='Camera driver: v4l2 (robust UVC /dev/video0) or astra (OpenNI 3D)')

    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Record raw camera video to MP4 dataset file on Pi')

    record_name_arg = DeclareLaunchArgument(
        'record_name', default_value='',
        description='Custom filename for recorded MP4 video')

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

    # ── Static TF: odom -> base_footprint (Khi không có encoder ESP32) ─
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
        condition=UnlessCondition(LaunchConfiguration('enable_esp32'))
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

    # ── Camera Driver (V4L2 USB Webcam DVD20) ─────────────────────────
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='camera',
        name='camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'MJPG',
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('enable_camera'), "' == 'true' and '", LaunchConfiguration('camera_driver'), "' == 'v4l2'"])
        )
    )

    # ── Camera Driver (Orbbec Astra Mini S 3D Camera via OpenNI2) ───
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

    camera_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        env=camera_env,
        parameters=[{
            'camera_name': 'camera',
            'vendor_id': '0x2bc5',
            'product_id': '0x0407',
            'color_width': LaunchConfiguration('color_width'),
            'color_height': LaunchConfiguration('color_height'),
            'color_fps': 30,
            'depth_width': 320,
            'depth_height': 240,
            'depth_fps': 30,
            'enable_color': True,
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_ir': False,
            'enable_point_cloud': False,
            'use_uvc_camera': False,
            'oni_log_level': 'none',
            'oni_log_to_console': False,
            'oni_log_to_file': False,
            'publish_tf': False,
            'camera_link_frame_id': 'camera_link',
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('enable_camera'), "' == 'true' and '", LaunchConfiguration('camera_driver'), "' == 'astra'"])
        )
    )

    # ── WiFi Camera Bridge (Nén JPEG gửi qua Wi-Fi) ─────────────────
    wifi_cam_bridge = Node(
        package='my_robot_bringup',
        executable='wifi_cam_bridge',
        name='wifi_cam_bridge',
        output='screen',
        parameters=[{
            'target_width': 320,
            'target_height': 240,
            'jpeg_quality': 50,
            'skip_frames': 2,
        }],
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )

    # ── ESP32 Hardware Bridge (Motor PID + Encoder Odom) ─────────────
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
        }],
        condition=IfCondition(LaunchConfiguration('enable_esp32'))
    )

    # ── CNN Driver (Autonomous Row Following) ────────────────────────
    cnn_driver = Node(
        package='my_robot_controller',
        executable='cnn_driver',
        name='cnn_driver_node',
        output='screen',
        parameters=[params_real],
        condition=IfCondition(LaunchConfiguration('enable_cnn'))
    )

    # ── Raw Video Recorder (Chỉ bật khi record:=true để ghi dataset thô) ─
    video_recorder = Node(
        package='my_robot_bringup',
        executable='video_recorder',
        name='video_recorder',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('camera_device'),
            'topic': '/camera/color/image_raw',
            'filename': LaunchConfiguration('record_name'),
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'direct_capture': True,
        }],
        condition=IfCondition(LaunchConfiguration('record'))
    )

    # ── RViz2 ────────────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    return LaunchDescription([
        serial_port_arg,
        camera_device_arg,
        esp32_port_arg,
        enable_esp32_arg,
        enable_camera_arg,
        enable_depth_arg,
        color_width_arg,
        color_height_arg,
        enable_cnn_arg,
        enable_rviz_arg,
        camera_driver_arg,
        record_arg,
        record_name_arg,
        robot_state_pub,
        joint_state_pub,
        static_odom_tf,
        lidar_node,
        v4l2_camera_node,
        camera_node,
        wifi_cam_bridge,
        esp32_bridge,
        video_recorder,
        cnn_driver,
        rviz2_node,
    ])
