import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    config_file = os.path.join(
        get_package_share_directory('navigation_stack'),
        'config',
        'slam_toolbox.yaml'
    )

    # Declare slam_toolbox as a LifecycleNode so we can control state transitions
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # After slam_node is in 'unconfigured', send CONFIGURE transition
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node is slam_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # After slam_node transitions to 'inactive', send ACTIVATE transition
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node is slam_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    # Small delay to wait for slam_toolbox to fully start before configuring
    delayed_configure = TimerAction(
        period=3.0,
        actions=[configure_event]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        slam_node,
        activate_on_inactive,
        delayed_configure,
    ])
