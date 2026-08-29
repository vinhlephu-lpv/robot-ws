from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bao',
    maintainer_email='bao@todo.todo',
    description='Control and Perception Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cnn_driver = my_robot_controller.cnn_driver:main',
            'data_collector = my_robot_controller.data_collector:main',
            'lidar_processor = my_robot_controller.lidar_processor:main',
            'gps_driver = my_robot_controller.gps_driver:main',
            'imu_driver = my_robot_controller.imu_driver:main',
            'bts7960_driver = my_robot_controller.bts7960_driver:main',
            'data_collection_driver = my_robot_controller.data_collection_driver:main',
            'telemetry_logger = my_robot_controller.telemetry_logger:main',
            'teleop_wasd = my_robot_controller.teleop_wasd:main',
            'plot_response = my_robot_controller.plot_response:main',
        ],
    },
)
