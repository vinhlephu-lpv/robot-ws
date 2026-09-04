from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_sensor_test'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vinh',
    maintainer_email='user@todo.todo',
    description='Dedicated package for testing real Camera and LiDAR sensors',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'sensor_diagnostics = my_sensor_test.sensor_diagnostics:main',
            'sensor_visualizer = my_sensor_test.sensor_visualizer:main',
            'check_imu = my_sensor_test.check_imu:main',
        ],
    },
)
