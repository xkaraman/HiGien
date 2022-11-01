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
    arguments = "0.0"

    lidar_static_transformer = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_transform_publisher',
            arguments = ['0.0', '0.0', '0.1', '3.14', '0', '0', 'base_link', 'laser']
        )

    # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    # serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') #for s2>
    # frame_id = LaunchConfiguration('frame_id', default='laser')
    # inverted = LaunchConfiguration('inverted', default='false')
    # angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    # scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')

    lidar_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch/sllidar_s2_launch.py'))
    )

#  DeclareLaunchArgument(
#             'serial_port',
#             default_value=serial_port,
#             description='Specifying usb port to connected lidar'),

#         DeclareLaunchArgument(
#             'serial_baudrate',
#             default_value=serial_baudrate,
#             description='Specifying usb port baudrate to connected lidar'),

#         DeclareLaunchArgument(
#             'frame_id',
#             default_value=frame_id,
#             description='Specifying frame_id of lidar'),

#         DeclareLaunchArgument(
#             'inverted',
#             default_value=inverted,
#             description='Specifying whether or not to invert scan data'),

# DeclareLaunchArgument(
#             'angle_compensate',
#             default_value=angle_compensate,
#             description='Specifying whether or not to enable angle_compensate of scan d>

#         DeclareLaunchArgument(
#             'scan_mode',
#             default_value=scan_mode,
#             description='Specifying scan mode of lidar'),

        # lidar_node = Node(
        #     package='sllidar_ros2',
        #     executable='sllidar_node',
        #     name='sllidar_node',
        #     parameters=[{'serial_port': serial_port, 
        #                  'serial_baudrate': serial_baudrate, 
        #                  'frame_id': frame_id,
        #                  'inverted': inverted, 
        #                  'angle_compensate': angle_compensate, 
        #                  'scan_mode': scan_mode}],
        #     output='screen')

    return LaunchDescription([
        lidar_static_transformer,
        lidar_launch_include
    ])