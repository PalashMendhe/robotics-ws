import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    
    # Robot description
    robot_description = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py')
            ]),

            launch_arguments={'gz_args': '-r ' + os.path.join(
                get_package_share_directory('robot_description'),
                'worlds', 'warehouse.sdf')
            }.items()),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Spawn robot in Gazebo
                Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
        '-name', 'my_robot',
        '-topic', '/robot_description',
        '-x', '0.0',
        '-y', '0.0', 
        '-z', '0.1',
    ],
    output='screen'
),



        # ROS-Gazebo bridge for topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
    '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
    '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
],
            output='screen'
        ),
    ])

