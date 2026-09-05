import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')
    arm_file = os.path.join(pkg_robot_description, 'urdf', 'arm.urdf.xacro')
    world_file = os.path.join(pkg_robot_description, 'worlds', 'large_warehouse.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_arg = LaunchConfiguration('x')
    y_arg = LaunchConfiguration('y')
    z_arg = LaunchConfiguration('z')
    yaw_arg = LaunchConfiguration('yaw')
    headless_arg = LaunchConfiguration('headless')

    # AMR URDF Description
    robot_description_cmd = Command(['xacro ', urdf_file])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)
    
    # Static Arm URDF Description
    arm_description_cmd = Command(['xacro ', arm_file])
    arm_description = ParameterValue(arm_description_cmd, value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        # Default spawn: AMR Home / Charging Pad (facing North toward Station 1)
        DeclareLaunchArgument(
            'x',
            default_value='-4.5',
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='-4.5',
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.08',
            description='Initial z position of the robot'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.9',
            description='Initial yaw orientation of the robot'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the gz server (no GUI)'
        ),

        # 1. Launch Gazebo Harmonic with large_warehouse.sdf
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ]),
            launch_arguments={
                'gz_args': [
                    '-r ', world_file,
                    IfElseSubstitution(
                        headless_arg,
                        if_value=' -s --headless-rendering',
                        else_value=''
                    ),
                ],
            }.items()
        ),

        # 2. Robot State Publisher (publishes robot TF tree)
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

        # 3. Spawn AMR on the Home Pad
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'my_robot',
                        '-string', robot_description_cmd,
                        '-world', 'large_warehouse',
                        '-x', x_arg,
                        '-y', y_arg,
                        '-z', z_arg,
                        '-Y', yaw_arg,
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # 3b. Spawn Static Arm 1 at Station 1
        TimerAction(
            period=4.5,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'station_1_arm',
                        '-string', arm_description_cmd,
                        '-world', 'large_warehouse',
                        '-x', '-4.684',
                        '-y', '0.5280',
                        '-z', '0.0',
                        '-Y', '3.14159', # Facing +X (North, over the shelf)
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # 3c. Spawn Static Arm 2 at Station 2
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'station_2_arm',
                        '-string', arm_description_cmd,
                        '-world', 'large_warehouse',
                        '-x', '-4.684',
                        '-y', '1.5280',
                        '-z', '0.0',
                        '-Y', '3.14159', # Facing +X (South, over the shelf)
                    ],
                    output='screen'
                ),
            ]
        ),
        # 3d. Spawn Static Arm 3 at Station 3
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'station_3_arm',
                        '-string', arm_description_cmd,
                        '-world', 'large_warehouse',
                        '-x', '-4.684',
                        '-y', '2.5280',
                        '-z', '0.0',
                        '-Y', '3.14159', # Facing +X (South, over the shelf)
                    ],
                    output='screen'
                ),
            ]
        ),

        # 4. TF Broadcaster from Odometry (odom -> base_footprint)
        Node(
            package='nav_nodes',
            executable='broadcaster_node',
            name='broadcaster_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 5. Topic Bridge (Gazebo Sim <-> ROS 2)
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
                '/camera/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera/rear/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rear/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen'
        ),

        # 6. ArUco Detector Node (starts once camera streams are live)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='nav_nodes',
                    executable='aruco_detector_node',
                    name='aruco_detector_node',
                    output='screen',
                    parameters=[{
                        'marker_size': 0.1125,
                        'dict_id': 0,  # DICT_4X4_50
                    }],
                ),
            ]
        ),
    ])
