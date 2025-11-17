#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('gemini_sonar_driver'),
            'config',
            'gemini_sonar.yaml'
        ]),
        description='Path to sonar configuration YAML file'
    )
    
    enable_rosbag_arg = DeclareLaunchArgument(
        'enable_rosbag',
        default_value='true',
        description='Enable ROS2 bag recording of sonar data'
    )
    
    bag_directory_arg = DeclareLaunchArgument(
        'bag_directory',
        default_value='~/gemini_bags',
        description='Directory for ROS2 bag files'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS logger level: debug, info, warn, error, fatal'
    )
    
    # Gemini sonar driver node
    gemini_node = Node(
        package='gemini_sonar_driver',
        executable='gemini_sonar_node',
        name='gemini_sonar_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'log_level': LaunchConfiguration('log_level')}
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        enable_rosbag_arg,
        bag_directory_arg,
        log_level_arg,
        gemini_node
    ])
