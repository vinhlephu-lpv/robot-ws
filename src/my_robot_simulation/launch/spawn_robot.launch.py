from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_pkg = FindPackageShare("my_robot_description")
    sim_pkg = FindPackageShare("my_robot_simulation")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                description_pkg,
                "urdf",
                "robot.urdf.xacro"
            ])
        ]),
        value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution([
                sim_pkg,
                "worlds",
                "empty.sdf"
            ])
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-z", "0.2"
        ]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ],
        output="screen"
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            PathJoinSubstitution([
                description_pkg,
                "config",
                "controllers.yaml"
            ])
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_robot,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[diff_drive_controller]
            )
        ),
    ])

