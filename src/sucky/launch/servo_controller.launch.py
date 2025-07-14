#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', 
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    # Servo controller node
    servo_controller_node = Node(
        package='sucky',
        executable='servo_controller.py',
        name='servo_controller',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout': 2.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        servo_controller_node
    ])
