"""
Launch file chạy hệ thống mô phỏng (Bước 1):
  - Gazebo Harmonic với vườn bắp
  - Robot State Publisher
  - RViz 2
  - Spawn robot
  - ROS-Gazebo Bridge
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_robot  = get_package_share_directory('my_robot_description')
    pkg_sim    = get_package_share_directory('my_robot_simulation')

    xacro_file   = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
    world_file   = os.path.join(pkg_sim,   'worlds', 'corn_field.sdf')
    rviz_file    = os.path.join(pkg_robot, 'rviz', 'display.rviz')

    robot_description = Command(['xacro "', xacro_file, '"'])

    # ── Gazebo Sim ────────────────────────────────────────────────────
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r "{world_file}"'}.items()
    )

    # ── Robot State Publisher ─────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # ── RViz 2 ────────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}]
    )

    # ── Spawn robot ───────────────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',  'my_robot',
            '-topic', 'robot_description',
            '-world', 'corn_field',
            '-x', '-1.5', '-y', '0.5', '-z', '0.2', '-Y', '0.0'
        ]
    )

    # ── ROS-Gazebo Bridge ─────────────────────────────────────────────
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
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        ]
    )

    return LaunchDescription([gz_sim, rsp, rviz2, spawn, bridge])
