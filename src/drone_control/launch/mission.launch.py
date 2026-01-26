import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- ARGUMENTS (Command Line Inputs) ---\
    target_lat_arg = DeclareLaunchArgument(
        'target_lat', 
        default_value='360.0',
        description='Target Latitude for the Drop Zone'
    )
    
    target_lon_arg = DeclareLaunchArgument(
        'target_lon', 
        default_value='360.0',
        description='Target Longitude for the Drop Zone'
    )

    # --- MAVROS (The Bridge) ---
    mavros_share = get_package_share_directory('mavros')

    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(mavros_share, 'launch', 'apm.launch')
        ),
        launch_arguments={
            'fcu_url': '/dev/ttyACM0',  # USB Connection to Pixhawk
            'gcs_url': 'udp://@',       # Bridge to Laptop Mission Planner via WiFi
            'tgt_system': '1',
            'tgt_component': '1',
            'log_output': 'screen',
            'respawn_mavros': 'true'
        }.items()
    )

    # --- VISION NODE (QR Detector) ---
    qr_detector_node = Node(
        package='vision',
        executable='qr_detector',
        name='qr_detector',
        output='log',
        emulate_tty=True
    )

    # --- PAYLOAD NODE (Servo Driver) ---
    payload_node = Node(
        package='payload',
        executable='payload_driver',
        name='payload_driver',
        output='screen',
        emulate_tty=True
    )

    # --- DRONE CONTROLLER  ---
    drone_controller_node = Node(
        package='drone_control',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        emulate_tty=True, 
        parameters=[{
            'target_lat': LaunchConfiguration('target_lat'),
            'target_lon': LaunchConfiguration('target_lon')
        }]
    )

    return LaunchDescription([
        target_lat_arg,
        target_lon_arg,
        mavros_launch,
        qr_detector_node,
        payload_node,
        drone_controller_node
    ])