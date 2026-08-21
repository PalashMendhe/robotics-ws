import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():

    arm_urdf = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf', 'arm.urdf.xacro'
    )

    arm_controllers_yaml = os.path.join(
        get_package_share_directory('robot_description'),
        'config', 'arm_controllers.yaml'
    )
    arm_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='arm',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', arm_urdf]), value_type=str),
            'use_sim_time': True
        }],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='arm',
        parameters=[arm_controllers_yaml],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/arm/controller_manager',
            '--controller-manager-timeout', '30',
            '--namespace', 'arm',
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/arm/controller_manager',
            '--controller-manager-timeout', '30',
            '--namespace', 'arm',
        ],
        output='screen'
    )
    

    return LaunchDescription([
        arm_rsp,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        #TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        #TimerAction(period=7.0, actions=[arm_controller_spawner]),
    ])