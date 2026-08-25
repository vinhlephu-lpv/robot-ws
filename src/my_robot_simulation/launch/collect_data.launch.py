"""
Launch file cho quy trình THU THẬP DỮ LIỆU TỰ ĐỘNG trong Gazebo:
  - Khởi chạy thế giới vườn bắp Gazebo Harmonic (hỗ trợ hàng cây uốn cong, gờ đất rung lắc)
  - Spawn robot & Bridge giao tiếp ROS2-Gazebo
  - Node `data_collection_driver`: Tự động di chuyển xe với lắc lái/rung lắc đung đưa
  - Node `data_collector`: Đăng ký nhận ảnh từ `/camera/image_raw` và lưu tự động vào dataset

Sử dụng:
  ros2 launch my_robot_simulation collect_data.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    pkg_robot = get_package_share_directory('my_robot_description')
    pkg_sim   = get_package_share_directory('my_robot_simulation')
    pkg_ctrl  = get_package_share_directory('my_robot_controller')

    xacro_file = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_sim,   'worlds', 'corn_field.sdf')

    robot_description = Command(['xacro "', xacro_file, '"'])

    # ── 1. Gazebo Sim ───────────────────────────────────────────────────
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r "{world_file}"'}.items()
    )

    # ── 2. Robot State Publisher ────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # ── 3. Spawn Robot ──────────────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',  'my_robot',
            '-topic', 'robot_description',
            '-world', 'corn_field',
            '-x', '-1.5', '-y', '0.4', '-z', '0.2', '-Y', '0.0'
        ]
    )

    # ── 4. ROS-Gazebo Bridge ────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
    )

    # ── 5. Data Collection Driver (Tự động di chuyển & lắc lái) ─────────
    driver_node = Node(
        package='my_robot_controller',
        executable='data_collection_driver',
        name='data_collection_driver_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'base_speed': 0.60, 'wobble_amplitude': 0.45}]
    )

    # ── 6. Data Collector (Tự động lưu ảnh) ────────────────────────────
    collector_node = Node(
        package='my_robot_controller',
        executable='data_collector',
        name='data_collector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'save_interval': 0.5,       # 2 FPS (mỗi 0.5s thu 1 frame)
            'only_when_moving': True
        }]
    )

    return LaunchDescription([gz_sim, rsp, spawn, bridge, driver_node, collector_node])
