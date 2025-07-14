#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', 
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    # Cyclone controller node
    cyclone_controller_node = Node(
        package='sucky',
        executable='cyclone_controller.py',
        name='cyclone_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout': 1.0,
            'reconnect_delay': 5.0
        }],
        remappings=[
            # Add any topic remappings if needed
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        cyclone_controller_node
    ])
