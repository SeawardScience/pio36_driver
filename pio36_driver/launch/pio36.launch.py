#!/usr/bin/env python3
"""
Launch file for PIO36 driver node

Starts the pio36 driver with configuration from example_config.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pio36_driver')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(pkg_dir, 'config', 'example_config.yaml'),
        description='Path to PIO36 configuration YAML file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the pio36 node'
    )
    
    # PIO36 driver node
    pio36_node = Node(
        package='pio36_driver',
        executable='pio36',
        name='pio36',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        pio36_node
    ])
