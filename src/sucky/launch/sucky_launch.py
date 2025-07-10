import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    bringup_pkg = get_package_share_directory('sucky')
    rviz_config_path = os.path.join(
        get_package_share_directory('sucky'),'config','rviz.rviz')

    # Load and Process Xacro
    xacro_file = os.path.join(bringup_pkg, 'urdf', 'robot.urdf.xacro')
    with open(xacro_file) as f:
        doc = xacro.parse(f)

    # Process the parsed Xacro document to generate the URDF XML
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Joint State Publisher for static RViz visualization - reduce output
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='log'  # Reduce console output
    )

    # Robot State Publisher - reduce output
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',  # Reduce console output
        parameters=[robot_description]
    )

    # Rviz - reduce update rate for better performance
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',  # Reduce console output
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        # Reduce priority to give more CPU to camera
        additional_env={'ROS_LOG_LEVEL': 'WARN'}
    )

    joy_params = os.path.join(get_package_share_directory('sucky'),'config','joystick.yaml')
    joystick_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': False}],
    )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel','/cmd_vel_joy')],
    )

    robot_controllers_path = os.path.join(get_package_share_directory('sucky'),'config','sucky_controllers.yaml')
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_path, {'use_sim_time': False}],
        output="both",
        #remappings=[('/diffbot_base_controller/odom', '/odom'),]
        
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': False}],
    )

    joint_state_publisher_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
        parameters=[{'use_sim_time': False}],
    )

    twist_mux_params = os.path.join(get_package_share_directory('sucky'),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
    )

    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    tim_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_tim_7xx.launch')

    sick_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='log',  # Keep log output to reduce CPU
        parameters=[{'use_sim_time': False}],
        arguments=[
            tim_launch_file_path,
            'tf_base_frame_id:=sick_link',
            'tf_publish_rate:=20.0',  # Reduce from 30 to 20 Hz
            'hostname:=192.168.0.1',
            'min_ang:=-1.22173',  # -70 degrees in radians
            'max_ang:=1.22173',   # 70 degrees in radians
        ]
    )

    ekf_params_file = os.path.join(get_package_share_directory('sucky'), 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='log',  # Reduce console output
        parameters=[ekf_params_file, {'use_sim_time': False}],
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sucky'),'launch','sucky_slam.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    battery_monitor_node = Node(
        package='sucky',
        executable='roboclaw_battery_monitor_shared.py',
        name='roboclaw_battery_monitor_shared',
        output='log',  # Reduce output
        parameters=[{
            'serial_port': '/dev/ttyACM0', 
            'address': 128,
            'publish_rate': 0.1, 
            'min_voltage': 22.0,
            'max_voltage': 29.4,
            'use_sim_time': False
        }]
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sucky'), 'launch', 'realsense_launch.py'
        )])
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        joystick_node,
        teleop_node,
        ros2_control_node,
        twist_mux,
        robot_controller_spawner,
        joint_state_publisher_spawner,
        sick_node,
        realsense_node,
        ekf_node,
        slam_node,
        battery_monitor_node,
    ])