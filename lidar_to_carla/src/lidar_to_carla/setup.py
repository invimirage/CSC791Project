from setuptools import setup
import os
from glob import glob


package_name = 'lidar_to_carla'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('lidar_to_carla/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='youremail@example.com',
    description='Bridge lidar data from ROS2 to Carla',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fetch_lidar = lidar_to_carla.fetch_lidar:main',
            'lidar_obj_detect = lidar_to_carla.lidar_obj_detect:main',
        ],
    },
)
