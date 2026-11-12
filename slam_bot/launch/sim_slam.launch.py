"""
SLAM in simulation: Gazebo (TurtleBot3 world), slam_toolbox, the wander node and RViz2.

    export TURTLEBOT3_MODEL=burger
    ros2 launch slam_bot sim_slam.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_bot = get_package_share_directory('slam_bot')

    auto_explore = LaunchConfiguration('auto_explore')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('auto_explore', default_value='true',
                              description='Run the wander node so the robot explores on its own'),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch', 'turtlebot3_world.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': os.path.join(slam_bot, 'config', 'slam_toolbox.yaml'),
            }.items()),

        Node(package='slam_bot', executable='wander', output='screen',
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(auto_explore)),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(slam_bot, 'config', 'slam.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(rviz)),
    ])
