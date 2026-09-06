import os

os.environ['GZ_IP'] = '127.0.0.1'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Full bringup for the random-station docking + delivery mission:

      1. warehouse.launch.py   — Gazebo (large_warehouse), AMR, 3 station
                                 arms (namespaced ros2_control), TF, bridge
      2. nav2.launch.py        — AMCL + Nav2 on the large_warehouse map
      3. station_arm_node x3   — /station_N/dock_arm pick-and-place services

    The mission itself is started separately so the random station/
    destination selection is user-triggered:

      ros2 run nav_nodes docking_mission

    NOTE: ekf.launch.py is intentionally NOT included — its
    robot_localization ekf_node publishes the odom->base_link TF, which
    fights broadcaster_node's odom->base_footprint in the same tree.
    broadcaster_node alone provides the odom->base_footprint TF Nav2 needs.
    """
    pkg_robot_description = get_package_share_directory('robot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_robot_description, 'launch', 'warehouse.launch.py')),
        launch_arguments={'headless': headless, 'use_sim_time': use_sim_time}.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_robot_description, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # One station arm node per station — each serves /station_N/dock_arm
    station_nodes = TimerAction(
        period=15.0,
        actions=[
            Node(package='nav_nodes', executable='station_arm_node',
                 name='station_1_arm_node', output='screen',
                 parameters=[{'station': 1, 'use_sim_time': use_sim_time}]),
            Node(package='nav_nodes', executable='station_arm_node',
                 name='station_2_arm_node', output='screen',
                 parameters=[{'station': 2, 'use_sim_time': use_sim_time}]),
            Node(package='nav_nodes', executable='station_arm_node',
                 name='station_3_arm_node', output='screen',
                 parameters=[{'station': 3, 'use_sim_time': use_sim_time}]),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo headless if true'),
        warehouse_launch,
        nav2_launch,
        station_nodes,
    ])
