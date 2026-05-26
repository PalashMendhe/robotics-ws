from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('week1_nodes')
    config_file = os.path.join(pkg_dir, 'config', 'sensor_params.yaml')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='500',
            description='Publishing frequency in ms'
        ),

        # Sensor subscriber
        Node(
            package='week1_nodes',
            executable='sensor_subscriber',
            name='sensor_subscriber',
            output='screen'
        ),

        # TF2 broadcaster
        Node(
            package='week1_nodes',
            executable='tf2_broadcaster',
            name='tf2_broadcaster',
            output='screen'
        ),

        # Custom publisher
        Node(
            package='week1_nodes',
            executable='custom_publisher',
            name='custom_publisher',
            output='screen'
        ),
    ])
