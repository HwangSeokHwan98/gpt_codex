from setuptools import setup
import os
from glob import glob

package_name = 'engineering_calculator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Example Maintainer',
    maintainer_email='example@example.com',
    description='Scientific calculator service for ROS2 (Foxy).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calculator_server = engineering_calculator.calculator_node:main',
        ],
    },
)
