import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    # Default parameters file
    default_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    default_bt_xml = os.path.join(pkg_nav, 'behavior_trees', 'navigate_with_smoothing.xml')

    declare_default_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=default_bt_xml,
        description='Full path to the behavior tree xml file to use'
    )

    # Navigation Bringup (Launch Planner, Controller, Recoveries, BT Navigator)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(navigation_launch)

    return ld
