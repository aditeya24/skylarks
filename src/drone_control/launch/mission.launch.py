import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. ARGUMENTS (Command Line Inputs) ---\
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

    # --- 2. MAVROS (The Bridge) ---
    mavros_share = get_package_share_directory('mavros')
    config_file = os.path.join(
        get_package_share_directory('drone_control'),
        'config',
        'mavros_config.yaml'
    )
    
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

    # --- 3. DRONE CONTROLLER (The Brain) ---
    drone_controller_node = Node(
        package='drone_control',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        emulate_tty=True, # Improved console logging formatting
        parameters=[{
            'target_lat': LaunchConfiguration('target_lat'),
            'target_lon': LaunchConfiguration('target_lon')
        }]
    )

    return LaunchDescription([
        target_lat_arg,
        target_lon_arg,
        mavros_launch,
        drone_controller_node
    ])