"""
SLAM on the real robot: Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) with an RPLIDAR and slam_toolbox.
The motor/encoder driver must publish odom -> base_link.

    ros2 launch slam_bot rpi5_slam.launch.py serial_port:=/dev/ttyUSB0

Run RViz on a laptop with the same ROS_DOMAIN_ID to keep load off the Pi.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_bot = get_package_share_directory('slam_bot')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),

        Node(package='rplidar_ros', executable='rplidar_node', output='screen',
             parameters=[{
                 'serial_port': LaunchConfiguration('serial_port'),
                 'serial_baudrate': 115200,
                 'frame_id': 'laser',
                 'angle_compensate': True,
             }]),

        # LiDAR mount relative to base_link; measure on the robot and update
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(slam_bot, 'config', 'slam_toolbox.yaml'),
            }.items()),
    ])
