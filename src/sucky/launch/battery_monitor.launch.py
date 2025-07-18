from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    # Get package directory
    pkg_share = FindPackageShare('sucky')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'battery_monitor.yaml'
    ])
    
    # Declare launch arguments for overrides
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM2',
        description='Serial port for roboclaw communication (shared with ROS2 control)'
    )
    
    enable_shutdown_arg = DeclareLaunchArgument(
        'enable_shutdown',
        default_value='true',
        description='Enable automatic shutdown on critical battery voltage'
    )
    
    shutdown_voltage_arg = DeclareLaunchArgument(
        'shutdown_voltage',
        default_value='20.5',
        description='Critical voltage threshold for automatic shutdown (V)'
    )
    
    shutdown_delay_arg = DeclareLaunchArgument(
        'shutdown_delay',
        default_value='30.0',
        description='Delay before shutdown execution (seconds)'
    )
    
    # Battery monitor node
    battery_monitor_node = Node(
        package='sucky',
        executable='battery_monitor.py',
        name='battery_monitor',
        output='screen',
        parameters=[
            config_file,
            {
                # Allow command-line overrides
                'serial_port': LaunchConfiguration('serial_port'),
                'enable_shutdown': LaunchConfiguration('enable_shutdown'),
                'shutdown_voltage': LaunchConfiguration('shutdown_voltage'),
                'shutdown_delay': LaunchConfiguration('shutdown_delay'),
            }
        ],
        remappings=[
            ('battery_voltage', 'battery_voltage'),
            ('diagnostics', 'diagnostics')
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        enable_shutdown_arg,
        shutdown_voltage_arg,
        shutdown_delay_arg,
        battery_monitor_node
    ])
