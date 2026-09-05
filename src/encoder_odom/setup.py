from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'encoder_odom'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bao',
    maintainer_email='baohi2k4@gmail.com',
    description='4-Wheel Quadrature Encoder Odometry Node with Cross-Validation and Anomaly Isolation',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'encoder_node = encoder_odom.encoder_node:main',
        ],
    },
)
