import os
import shutil
import tempfile
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def make_arm_nodes(arm_urdf, controllers, use_sim_time, ns, name, x, y, z, yaw):
    """Helper — returns (rsp_node, spawn_action, controllers_action) for one arm."""

    # Each arm gets its own URDF baked with its namespace + controllers path
    # NOTE: the same Command substitution is reused for the spawn below. This
    # matches the bot's spawn method (-string) and avoids the latched-topic
    # race: RSP publishes robot_description exactly ONCE (transient_local) at
    # startup, and `create -topic` subscribes WITHOUT transient_local — so any
    # subscriber joining after that single publication never receives it and
    # waits forever ("Waiting messages on topic"). Spawning via -string has no
    # such dependency.
    arm_desc_cmd = Command([
        'xacro ', arm_urdf,
        ' controllers_file:=', controllers,
        ' arm_namespace:=', ns,
    ])
    arm_desc = ParameterValue(arm_desc_cmd, value_type=str)

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

    # Spawn the arm model — delay so the Gazebo world is ready. The URDF is
    # passed inline (-string) exactly like the bot; do NOT switch to -topic
    # (see note above on the latched robot_description race).
    spawn = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',  name,
                    '-string', arm_desc_cmd,
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

    # Controller spawners — they poll the controller_manager until it appears,
    # so the timeout must comfortably exceed world-load + arm-spawn time
    # (observed up to ~70 s on slow machines; spawner timer alone at 22 s is
    # NOT enough — spawners died with "controller manager not available").
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
                    '--controller-manager-timeout', '120',
                    # Override the CM's inherited ROS args: gz_ros2_control adds
                    # the '-p use_sim_time:=true' shorthand which gets mangled
                    # ('--params-file -p') when forwarded to controller nodes.
                    '--controller-ros-args', '--ros-args -p use_sim_time:=true',
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
                    '--controller-manager-timeout', '120',
                    # Override the CM's inherited ROS args: gz_ros2_control adds
                    # the '-p use_sim_time:=true' shorthand which gets mangled
                    # ('--params-file -p') when forwarded to controller nodes.
                    '--controller-ros-args', '--ros-args -p use_sim_time:=true',
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
                    '--controller-manager-timeout', '120',
                    # Override the CM's inherited ROS args: gz_ros2_control adds
                    # the '-p use_sim_time:=true' shorthand which gets mangled
                    # ('--params-file -p') when forwarded to controller nodes.
                    '--controller-ros-args', '--ros-args -p use_sim_time:=true',
                ],
                output='screen'
            ),
        ]
    )

    return rsp, spawn, ctrl


def generate_launch_description():
    pkg_path  = get_package_share_directory('robot_description')
    arm_urdf  = os.path.join(pkg_path, 'urdf', 'arm.urdf.xacro')

    # IMPORTANT: the controllers yaml path baked into the arm URDF must NOT
    # contain the substring "robot_description". The namespaced controller
    # manager (Jazzy) strips every ROS arg containing that substring when it
    # forwards its own args to controller nodes — leaving a dangling
    # --params-file flag that pairs with the following '-p use_sim_time:=true'
    # and every controller fails with
    #   Couldn't parse params file: '--params-file -p'
    # (the installed share path is .../robot_description/... hence the copy).
    controllers_src = os.path.join(pkg_path, 'config', 'arm_controllers.yaml')
    controllers = os.path.join(tempfile.gettempdir(), 'arm_controllers.yaml')
    shutil.copyfile(controllers_src, controllers)

    # Set GZ_SIM_RESOURCE_PATH at Python parse time so Gazebo subprocess inherits
    # it before the world is loaded. The models/ directory contains the ArUco
    # marker models referenced via model://aruco_marker_N in multiroom.sdf.
    models_path = os.path.join(pkg_path, 'models')
    existing    = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        models_path + (':' + existing if existing else '')
    )

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

        # 2. Arm RSPs
        arm1_rsp,
        arm2_rsp,

        # 3. Spawn both arms after Gazebo world is ready
        arm1_spawn,
        arm2_spawn,

        # 4. Controllers for both arms
        arm1_ctrl,
        arm2_ctrl,

        # 5. ArUco detector — starts after cameras are publishing (t=6 s)
        TimerAction(period=6.0, actions=[
            Node(
                package='nav_nodes',
                executable='aruco_detector_node',
                name='aruco_detector_node',
                output='screen',
                parameters=[{
                    # Detectable marker = outer edge of the black border.
                    # Panel face is 0.15 m, texture has a 1-module white quiet
                    # zone around a 6-module marker -> marker spans 6/8 of the
                    # face = 0.15 * 0.75 = 0.1125 m. Must match the texture.
                    'marker_size': 0.1125,
                    'dict_id':     0,     # DICT_4X4_50
                }],
            ),
        ]),
    ])




