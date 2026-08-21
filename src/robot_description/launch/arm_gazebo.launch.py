import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'arm.urdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'arm_controllers.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Bake absolute controllers_file path into the URDF via xacro arg
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' controllers_file:=', controllers_file]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        # Launch a dedicated Gazebo instance for the arm (empty world)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': '-r empty.sdf',
                'on_exit_shutdown': 'true',
            }.items()
        ),

        # Clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # Robot state publisher for the arm
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arm_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/robot_description', '/arm_description')],
            output='screen'
        ),

        # Spawn arm after Gazebo is ready (5s delay)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'robotic_arm',
                        '-topic', '/arm_description',
                        '-x', '0.0', '-y', '0.0', '-z', '0.0',
                    ],
                    output='screen'
                ),
            ]
        ),

        # Spawn controllers after gz_ros2_control has time to start (18s)
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--param-file', controllers_file,
                        '--controller-manager-timeout', '30',
                    ],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'arm_controller',
                        '--param-file', controllers_file,
                        '--controller-manager-timeout', '30',
                    ],
                    output='screen'
                ),
            ]
        ),
    ])
