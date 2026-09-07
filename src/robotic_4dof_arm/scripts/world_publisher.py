#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

class WorldPublisher(Node):
    def __init__(self):
        super().__init__('world_marker_publisher')

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(MarkerArray, '/world_markers', qos)

        # Robot base spawn height in Gazebo world coordinates
        self.robot_base_z = 0.0

        self.markers = self.create_world_markers()
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()
        self.get_logger().info('World marker publisher active on /world_markers')

    def create_world_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        # 1. Docked AMR Chassis / Tray
        m = Marker()
        m.header.frame_id = 'world'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = -0.384
        m.pose.position.y = 0.300
        m.pose.position.z = 0.090
        m.pose.orientation.w = 1.0
        m.scale.x = 0.35
        m.scale.y = 0.32
        m.scale.z = 0.18
        m.color.r = 0.3
        m.color.g = 0.3
        m.color.b = 0.5
        m.color.a = 0.85
        marker_array.markers.append(m)

        # 2. Target Box (Floor parcel)
        m = Marker()
        m.header.frame_id = 'world'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = -0.484
        m.pose.position.y = -0.0893
        m.pose.position.z = 0.055
        m.pose.orientation.w = 1.0
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.r = 1.0
        m.color.g = 0.3
        m.color.b = 0.3
        m.color.a = 0.95
        marker_array.markers.append(m)

        # 3. Spawn Marker (Outer Ring on Floor)
        m = Marker()
        m.header.frame_id = 'world'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = -0.484
        m.pose.position.y = -0.0893
        m.pose.position.z = 0.005
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.01
        m.color.r = 0.2
        m.color.g = 0.4
        m.color.b = 0.9
        m.color.a = 0.9
        marker_array.markers.append(m)

        # 4. Spawn Marker (Inner Ring on Floor)
        m = Marker()
        m.header.frame_id = 'world'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = -0.484
        m.pose.position.y = -0.0893
        m.pose.position.z = 0.007
        m.pose.orientation.w = 1.0
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.01
        m.color.r = 0.95
        m.color.g = 0.95
        m.color.b = 0.95
        m.color.a = 1.0
        marker_array.markers.append(m)

        # 5. Destined Marker (Outer Ring at AMR Tray)
        m = Marker()
        m.header.frame_id = 'world'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = -0.384
        m.pose.position.y = 0.300
        m.pose.position.z = 0.005
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.01
        m.color.r = 0.15
        m.color.g = 0.85
        m.color.b = 0.35
        m.color.a = 0.9
        marker_array.markers.append(m)

        # 6. Destined Marker (Inner Ring at AMR Tray)
        m = Marker()
        m.header.frame_id = 'world'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = -0.384
        m.pose.position.y = 0.300
        m.pose.position.z = 0.007
        m.pose.orientation.w = 1.0
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.01
        m.color.r = 0.95
        m.color.g = 0.95
        m.color.b = 0.95
        m.color.a = 1.0
        marker_array.markers.append(m)

        return marker_array

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = now
        self.publisher.publish(self.markers)

def main(args=None):
    rclpy.init(args=args)
    node = WorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

