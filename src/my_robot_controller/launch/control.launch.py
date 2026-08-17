import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ctrl = get_package_share_directory('my_robot_controller')
    params_file = os.path.join(pkg_ctrl, 'config', 'params_sim.yaml')

    cnn_driver = Node(
        package='my_robot_controller',
        executable='cnn_driver',
        name='cnn_driver_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    data_collector = Node(
        package='my_robot_controller',
        executable='data_collector',
        name='data_collector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([cnn_driver, data_collector])
