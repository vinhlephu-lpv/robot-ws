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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_robot  = get_package_share_directory('my_robot_description')
    pkg_sim    = get_package_share_directory('my_robot_simulation')

    xacro_file   = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
    world_file   = os.path.join(pkg_sim,   'worlds', 'corn_field.sdf')
    rviz_file    = os.path.join(pkg_robot, 'rviz', 'display.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz 2'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-1.5', description='Spawn X position')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.75', description='Spawn Y position (lane center is 0.5, 0.75 is offset by 0.25m)')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='-0.20', description='Spawn Yaw orientation in radians (-0.20 is ~-11.5 deg offset)')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

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
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
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
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.2',
            '-Y', spawn_yaw
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

    return LaunchDescription([use_rviz_arg, spawn_x_arg, spawn_y_arg, spawn_yaw_arg, gz_sim, rsp, rviz2, spawn, bridge])
