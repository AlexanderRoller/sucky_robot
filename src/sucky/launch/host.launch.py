#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the rviz config file
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(get_package_share_directory('sucky'), 
                                   'config', 'rviz.rviz'),
        description='Full path to the RViz config file')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(rviz_node)

    return ld
