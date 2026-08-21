import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_arg = LaunchConfiguration('world')
    x_arg = LaunchConfiguration('x')
    y_arg = LaunchConfiguration('y')
    z_arg = LaunchConfiguration('z')
    yaw_arg = LaunchConfiguration('yaw')

    # Robot description command
    robot_description_cmd = Command(['xacro ', urdf_file])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # Path to world file
    world_file = PathJoinSubstitution([pkg_robot_description, 'worlds', world_arg])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='multiroom.sdf',
            description='World file name (inside worlds/) or full path to world file'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='2.2',
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.8',
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.1',
            description='Initial z position of the robot'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial yaw orientation of the robot'
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ]),
            launch_arguments={'gz_args': ['-r ', world_file]}.items()
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # Spawn robot in Gazebo after a delay to ensure Gazebo world is ready
        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'my_robot',
                        '-string', robot_description_cmd,
                        '-x', x_arg,
                        '-y', y_arg,
                        '-z', z_arg,
                        '-Y', yaw_arg,
                    ],
                    output='screen'
                ),
            ]
        ),

        # TF Broadcaster from Odometry
        Node(
            package='nav_nodes',
            executable='broadcaster_node',
            name='broadcaster_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ROS-Gazebo bridge for topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen'
        ),
    ])
