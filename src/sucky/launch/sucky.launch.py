import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Package directories and config paths
    bringup_pkg = get_package_share_directory('sucky')
    
    # Load and process URDF from Xacro
    xacro_file = os.path.join(bringup_pkg, 'urdf', 'robot.urdf.xacro')
    with open(xacro_file) as f:
        doc = xacro.parse(f)
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Configuration file paths
    joy_params = os.path.join(bringup_pkg, 'config', 'joystick.yaml')
    robot_controllers_path = os.path.join(bringup_pkg, 'config', 'sucky_controllers.yaml')
    twist_mux_params = os.path.join(bringup_pkg, 'config', 'twist_mux.yaml')
    ekf_params_file = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    
    # SICK LiDAR configuration
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    tim_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_tim_7xx.launch')

    # ============================================================================
    # CORE NODES (Robot Description and State Publishers)
    # ============================================================================
    
    # Robot State Publisher - publishes robot description and transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # Joint State Publisher for static visualization
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='log',
        parameters=[{'use_sim_time': False}]
    )

    # ============================================================================
    # CONTROL SYSTEM (Hardware Interface and Controllers)
    # ============================================================================
    
    # ROS2 Control Node - hardware interface
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_path, {'use_sim_time': False}],
        output="both"
    )

    # Joint State Broadcaster - publishes joint states from hardware
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': False}]
    )

    # Differential Drive Controller - controls robot movement
    diffbot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': False}]
    )

    # ============================================================================
    # INPUT DEVICES (Joystick and Teleop)
    # ============================================================================
    
    # Joystick Driver
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params, {'use_sim_time': False}],
        output='log'
    )

    # Teleop Twist Joy - converts joystick to cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        output='log'
    )

    # Custom Sucky Joy Controller - handles cyclone and door controls
    sucky_joy_node = Node(
        package='sucky',
        executable='sucky_joy.py',
        name='sucky_joy',
        output='log',
        parameters=[joy_params, {'use_sim_time': False}]
    )

    # Twist Mux - arbitrates between different cmd_vel sources
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diffbot_base_controller/cmd_vel_unstamped')],
        output='log'
    )

    # ============================================================================
    # SENSORS (LiDAR, Camera, Battery)
    # ============================================================================
    
    # SICK TiM LiDAR
    sick_lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='log',
        parameters=[{'use_sim_time': False}],
        arguments=[
            tim_launch_file_path,
            'tf_base_frame_id:=sick_link',
            'cloud_topic:=sick_cloud',
            'tf_publish_rate:=20.0',
            'hostname:=192.168.0.1',
            'min_ang:=-1.22173',  # -70 degrees
            'max_ang:=1.22173',   # 70 degrees
        ]
    )

    # RealSense Camera
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_pkg, 'launch', 'realsense.launch.py'
        )])
    )

    # Battery Monitor
    battery_monitor_node = Node(
        package='sucky',
        executable='battery_monitor.py',
        name='battery_monitor',
        output='log',
        parameters=[{
            'serial_port': '/dev/ttyACM2',
            'address': 128,
            'publish_rate': 0.1,
            'min_voltage': 22.0,
            'max_voltage': 29.4,
            'use_sim_time': False
        }]
    )

    # ============================================================================
    # LOCALIZATION AND MAPPING
    # ============================================================================
    
    # Extended Kalman Filter for sensor fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='log',
        parameters=[ekf_params_file, {'use_sim_time': False}]
    )

    # SLAM 
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_pkg, 'launch', 'slam.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # ============================================================================
    # ACTUATORS AND CUSTOM HARDWARE
    # ============================================================================
    
    # Arduino Controller - manages cyclone and door actuators
    arduino_controller_node = Node(
        package='sucky',
        executable='arduino_controller.py',
        name='arduino_controller',
        output='log',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'timeout': 2.0,
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        # Core nodes - robot description and state publishers (highest priority)
        robot_state_publisher,
        joint_state_publisher,
        
        # Control system - hardware interface and controllers
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diffbot_controller_spawner,
        
        # Input devices - joystick and teleop
        joystick_node,
        teleop_node,
        sucky_joy_node,
        twist_mux_node,
        
        # Sensors - LiDAR, camera, battery monitoring
        sick_lidar_node,
        realsense_node,
        battery_monitor_node,
        
        # Localization and mapping - depends on sensors
        ekf_node,
        slam_node,
        
        # Custom hardware actuators
        arduino_controller_node,
    ])