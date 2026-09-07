from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def should_accept_stamp(msg_nanos, last_stamp_nanos):
    """
    Decide whether an odom stamp should be forwarded to TF.

    Zero means Gazebo is not yet running / clock not synced; non-increasing
    stamps are duplicates or out-of-order messages that would trigger
    TF_OLD_DATA warnings in AMCL (its motion model then goes inconsistent).
    Pure function so the policy is unit-testable without a live node.
    """
    if msg_nanos == 0:
        return False
    return msg_nanos > last_stamp_nanos


class broadcaster_node(Node):
    def __init__(self):
        super().__init__('broadcaster_node')
        self.get_logger().info('TF broadcaster node started')

        # Use QoS compatible with ros_gz_bridge (Best Effort or Reliable)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/model/my_robot/odometry',
            self.odom_callback,
            qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_stamp_nanos = 0
        self.get_logger().info(
            'Subscribed to /model/my_robot/odometry -> broadcasting '
            'odom to base_footprint')

    def odom_callback(self, msg):
        # Use the actual odometry timestamp so AMCL's motion model stays consistent.
        # Enforce monotonically increasing timestamps to prevent TF_OLD_DATA warnings.
        msg_nanos = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if not should_accept_stamp(msg_nanos, self.last_stamp_nanos):
            return  # zero stamp (clock not synced) or duplicate / out-of-order
        self.last_stamp_nanos = msg_nanos

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = broadcaster_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
