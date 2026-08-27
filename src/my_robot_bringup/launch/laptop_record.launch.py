"""
Launch file: Thu thập dữ liệu Dataset trực tiếp qua RViz (Chạy trên Laptop đặt trên xe).

Bao gồm:
  - Robot State Publisher + Joint State Publisher: Hiển thị mô hình 3D xe trên RViz.
  - Astra Camera Node (hoặc USB V4L2): Mở Camera USB cắm vào laptop (Astra Mini S hoặc Webcam).
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
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
        description='Cổng camera USB (/dev/video2, auto)')

    width_arg = DeclareLaunchArgument(
        'width', default_value='640',
        description='Chiều rộng khung hình')

    height_arg = DeclareLaunchArgument(
        'height', default_value='480',
        description='Chiều cao khung hình')

    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30.0',
        description='FPS ghi video')

    interval_arg = DeclareLaunchArgument(
        'interval', default_value='0.2',
        description='Khoảng cách thời gian tách frame (giây) vào thư mục imgs')

    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', default_value='true',
        description='Mở giao diện RViz2')

    # ── Robot Model Nodes ──────────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
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

    # ── Tự động khởi động driver Camera USB ngoài ──────────────────────────
    # Kiểm tra xem có camera USB Astra Mini S (2bc5:0407) cắm vào máy không
    has_astra = False
    try:
        out = subprocess.check_output(['lsusb'], text=True, stderr=subprocess.DEVNULL)
        if '2bc5:' in out:
            has_astra = True
    except Exception:
        pass

    launch_entities = [
        name_arg,
        device_arg,
        width_arg,
        height_arg,
        fps_arg,
        interval_arg,
        open_rviz_arg,
        robot_state_pub,
        joint_state_pub,
        camera_recorder_node,
        rviz2_node,
    ]

    if has_astra:
        # Tự động nạp gói astra_camera từ /home/vinh/astra_ws nếu cần
        astra_install = '/home/vinh/astra_ws/install/astra_camera'
        if os.path.exists(astra_install):
            os.environ['AMENT_PREFIX_PATH'] = astra_install + ':' + os.environ.get('AMENT_PREFIX_PATH', '')

        try:
            astra_pkg = get_package_share_directory('astra_camera')
            astra_xml = os.path.join(astra_pkg, 'launch', 'astra.launch.xml')
            astra_node = IncludeLaunchDescription(
                AnyLaunchDescriptionSource(astra_xml),
                launch_arguments={
                    'enable_color': 'true',
                    'enable_depth': 'false',
                    'enable_ir': 'false',
                    'color_width': '640',
                    'color_height': '480',
                    'color_fps': '30',
                }.items()
            )
            launch_entities.insert(0, astra_node)
        except Exception:
            pass

    return LaunchDescription(launch_entities)
