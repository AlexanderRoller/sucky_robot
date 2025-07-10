from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for roboclaw communication (shared with ROS2 control)'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='128',
        description='Roboclaw address'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='0.2',
        description='Rate to publish battery voltage (Hz) - very slow to minimize interference'
    )
    
    min_voltage_arg = DeclareLaunchArgument(
        'min_voltage',
        default_value='22.0',
        description='Minimum battery voltage threshold (V)'
    )
    
    max_voltage_arg = DeclareLaunchArgument(
        'max_voltage',
        default_value='29.4',
        description='Maximum battery voltage threshold (V)'
    )
    
    # Roboclaw battery monitor node (shared access version)
    battery_monitor_node = Node(
        package='sweepy_hardware_drivers',
        executable='roboclaw_battery_monitor_shared.py',
        name='roboclaw_battery_monitor_shared',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'address': LaunchConfiguration('address'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'min_voltage': LaunchConfiguration('min_voltage'),
            'max_voltage': LaunchConfiguration('max_voltage'),
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        address_arg,
        publish_rate_arg,
        min_voltage_arg,
        max_voltage_arg,
        battery_monitor_node
    ])
