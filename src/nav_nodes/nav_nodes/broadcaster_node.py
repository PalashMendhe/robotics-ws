import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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
        self.get_logger().info('Subscribed to /model/my_robot/odometry -> broadcasting odom to base_footprint')

    def odom_callback(self, msg):
        # Use the actual odometry timestamp so AMCL's motion model stays consistent.
        # Enforce monotonically increasing timestamps to prevent TF_OLD_DATA warnings.
        msg_nanos = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if msg_nanos == 0:
            return  # Gazebo not yet running / clock not synced

        if msg_nanos <= self.last_stamp_nanos:
            return  # Drop duplicate or out-of-order message
        self.last_stamp_nanos = msg_nanos

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
        transform.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_footprint'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = broadcaster_node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()