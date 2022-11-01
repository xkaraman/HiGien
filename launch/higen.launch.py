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

# TODO argument for ttyUSB for disinfection mechanicm USB

def generate_launch_description():
    lidar_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('higien'),
                'launch/lidar.launch.py'))
    )

    zed_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('higien'),
                'launch/zed.launch.py'))
    )


    return LaunchDescription([
        lidar_launch_include,
        zed_launch_include
    ])