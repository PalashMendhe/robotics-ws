import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    arm_urdf = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf', 'arm.urdf.xacro'
    )

    arm_controllers_yaml = os.path.join(
        get_package_share_directory('robot_description'),
        'config', 'arm_controllers.yaml'
    )

    # Robot state publisher for the arm (separate namespace so it does not
    # clash with the differential-drive robot's robot_state_publisher)
    arm_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_state_publisher',
        namespace='arm',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', arm_urdf,
                         ' controllers_file:=', arm_controllers_yaml]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn the arm into the already-running Gazebo simulation
    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'arm',
            '-topic', '/arm/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ],
        output='screen'
    )

    # Controller spawners — staggered to give Gazebo time to load the arm
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--param-file', arm_controllers_yaml,
                    '--controller-manager-timeout', '30',
                ],
                output='screen'
            )
        ]
    )

    arm_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'arm_controller',
                    '--param-file', arm_controllers_yaml,
                    '--controller-manager-timeout', '30',
                ],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'gripper_controller',
                    '--param-file', arm_controllers_yaml,
                    '--controller-manager-timeout', '30',
                ],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        arm_rsp,
        spawn_arm,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])

