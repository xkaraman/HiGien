#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch.actions import DeclareLaunchArgument
# from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    zed_static_transformer = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_static_transform_publisher',
            arguments = ['0.1', '0.0', '0.7', '0.0', '0', '0', 'base_link', 'camera_link']
        )

    zed_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch/zed2.launch.py'))
    )

    return LaunchDescription([
        zed_static_transformer,
        zed_launch_include
    ])