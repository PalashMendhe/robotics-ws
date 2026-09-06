import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_desc = get_package_share_directory('robot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Default nav2_params.yaml inside robot_description/config/
    default_nav2_params = os.path.join(
        pkg_robot_desc,
        'config',
        'nav2_params.yaml'
    )

    default_map_file = os.path.join(pkg_robot_desc, 'maps', 'large_warehouse.yaml')

    nav2_bringup_launch = os.path.join(
        pkg_nav2_bringup,
        'launch',
        'bringup_launch.py'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        nav2_bringup
    ])
