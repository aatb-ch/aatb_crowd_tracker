#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('aatb_crowd_tracker')
    
    # Declare launch arguments - without defaults to avoid overriding YAML
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'crowd_tracker.yaml']),
        description='Path to configuration file'
    )
    
    # Create the node - only load the config file, no overrides
    crowd_tracker_node = Node(
        package='aatb_crowd_tracker',
        executable='crowd_tracker_node',
        name='crowd_tracker',
        parameters=[
            # Load YAML config file only
            LaunchConfiguration('config_file'),
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        config_file_arg,
        crowd_tracker_node
    ])