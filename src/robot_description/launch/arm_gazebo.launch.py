import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def make_arm_nodes(arm_urdf, controllers, use_sim_time, ns, name, x, y, z, yaw):
    """Helper — returns (rsp_node, spawn_action, controllers_action) for one arm."""

    # Each arm gets its own URDF baked with its namespace + controllers path
    arm_desc = ParameterValue(
        Command([
            'xacro ', arm_urdf,
            ' controllers_file:=', controllers,
            ' arm_namespace:=', ns,
        ]),
        value_type=str
    )

    # Robot state publisher in the arm's own namespace
    # → publishes to /<ns>/robot_description (picked up by gz_ros2_control)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_state_publisher',
        namespace=ns,
        parameters=[{
            'robot_description': arm_desc,
            'use_sim_time':      use_sim_time,
        }],
        output='screen'
    )

    # Spawn the arm model — delay so the RSP is fully up
    spawn = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',  name,
                    '-topic', f'/{ns}/robot_description',
                    '-world', 'multiroom',
                    '-x',     x,
                    '-y',     y,
                    '-z',     z,
                    '-Y',     yaw,
                ],
                output='screen'
            ),
        ]
    )

    cm = f'/{ns}/controller_manager'

    # Controller spawners — wait for gz_ros2_control inside Gazebo to boot
    ctrl = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', cm,
                    '--param-file', controllers,
                    '--controller-manager-timeout', '30',
                ],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'arm_controller',
                    '--controller-manager', cm,
                    '--param-file', controllers,
                    '--controller-manager-timeout', '30',
                ],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'gripper_controller',
                    '--controller-manager', cm,
                    '--param-file', controllers,
                    '--controller-manager-timeout', '30',
                ],
                output='screen'
            ),
        ]
    )

    return rsp, spawn, ctrl


def generate_launch_description():
    pkg_path  = get_package_share_directory('robot_description')
    arm_urdf  = os.path.join(pkg_path, 'urdf', 'arm.urdf.xacro')
    controllers = os.path.join(pkg_path, 'config', 'arm_controllers.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Gazebo + Differential Drive Bot ─────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'world':        'multiroom.sdf',
            'x':            '2.2',
            'y':            '0.8',
            'z':            '0.1',
            'yaw':          '0.0',
        }.items()
    )

    # ── Arm 1 — Shelf 1 (Room 1, same pose old arm_1 had) ───────────────────
    # Original pose: x=0.3 y=0.3 z=0.4  yaw=0
    arm1_rsp, arm1_spawn, arm1_ctrl = make_arm_nodes(
        arm_urdf, controllers, use_sim_time,
        ns='arm1', name='robotic_arm_1',
        x='0.75', y='0.3', z='0.4', yaw='0.0'
        
    )

    # ── Arm 2 — Shelf 2 (Room 3, same pose old arm_2 had) ───────────────────
    # Original pose: x=6.9 y=5.8 z=0.4  yaw=3.14159 (facing opposite direction)
    arm2_rsp, arm2_spawn, arm2_ctrl = make_arm_nodes(
        arm_urdf, controllers, use_sim_time,
        ns='arm2', name='robotic_arm_2',
        x='6.45', y='5.8', z='0.4', yaw='3.14159'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        # 1. Gazebo world + differential drive bot
        gazebo_launch,

        # 2. Arm 1 RSP  (shelf 1)
        arm1_rsp,
        # 3. Arm 2 RSP  (shelf 2)
        arm2_rsp,

        # 4. Spawn both arms after Gazebo world is ready
        arm1_spawn,
        arm2_spawn,

        # 5. Controllers for both arms
        arm1_ctrl,
        arm2_ctrl,
    ])




