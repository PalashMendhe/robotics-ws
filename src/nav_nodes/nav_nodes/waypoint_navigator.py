import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion  

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Publisher and subscriber — you fill the topic names
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/my_robot/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/my_robot/odometry', self.odom_callback, 10)
        
        # Waypoints as list of (x, y) tuples — hardcode 3 for now
        self.waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]
        self.current_waypoint_idx = 0
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Control params — you pick values based on our discussion
        self.Kp = 0.1
        self.linear_vel = 0.5
        self.goal_threshold = 0.1
        self.heading_threshold = 0.1
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.odom_received = False

    def odom_callback(self, msg):
        # Extract x, y from msg.pose.pose.position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Extract theta from quaternion — use msg.pose.pose.orientation
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.odom_received = True
        pass

    def control_loop(self):
        
        if not self.odom_received:
            return
        
        current_waypoint = self.waypoints[self.current_waypoint_idx]
        goal_x, goal_y = current_waypoint
        distance_to_goal = math.sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)
        bearing_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
        heading_error = bearing_to_goal - self.theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  

        if abs(heading_error) > self.heading_threshold:
            angular_vel = self.Kp * heading_error
            linear_vel = 0.0
        else:
            angular_vel = 0.0
            linear_vel = self.linear_vel

        if distance_to_goal < self.goal_threshold:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('All waypoints reached!')
                self.cmd_vel_pub.publish(Twist())
                self.timer.cancel()
                return
        
        # Publish velocities and other values to cmd_vel
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_vel
        cmd_vel_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()