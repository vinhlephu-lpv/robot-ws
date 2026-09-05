from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_navigation'

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
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bao',
    maintainer_email='bao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'costmap_node = my_robot_navigation.costmap_node:main',
            'cancel_nav = my_robot_navigation.cancel_nav:main',
            'gps_waypoint_follower = my_robot_navigation.gps_waypoint_follower:main',
        ],
    },
)
