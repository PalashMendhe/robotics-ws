import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from arm_description.srv import GetArmIK
import math


'''(x, y, z) target
     ↓
θ1 = atan2(y, x)                    → arm_base_joint
     ↓
r = sqrt(x² + y²) - wrist_length    → horizontal reach after subtracting wrist
z_eff = z - mount_height            → vertical reach from shoulder
     ↓
cos(θ3) = (r² + z² - L1² - L2²) / (2·L1·L2)
θ3 = atan2(sqrt(1 - cos²θ3), cosθ3) → forearm_joint
     ↓
θ2 = atan2(z_eff, r) - atan2(L2·sinθ3, L1 + L2·cosθ3) → upper_arm_joint
     ↓
θ4 = 0 (wrist keeps gripper level)  → wrist_joint'''

L1 = 0.17        # upper arm
L2 = 0.13        # forearm
MOUNT_HEIGHT = 0.1575  # ground to shoulder joint
WRIST_LENGTH = 0.128   # wrist + connector + gripper base

class ArmIK(Node):
    def __init__(self):
        super().__init__('arm_ik')
        self.publisher1 = self.create_publisher(Float64, '/arm_base_joint/cmd_pos', 10)
        self.publisher2 = self.create_publisher(Float64, '/upper_arm_joint/cmd_pos', 10)
        self.publisher3 = self.create_publisher(Float64, '/forearm_joint/cmd_pos', 10)
        self.publisher4 = self.create_publisher(Float64, '/wrist_joint/cmd_pos', 10)
        self.srv = self.create_service(GetArmIK, 'get_arm_ik', self.ik_callback)
    
    def ik_callback(self, request, response):
        x = request.x
        y = request.y
        z = request.z

        # compute inverse kinematics
        base_angle = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - WRIST_LENGTH
        z_eff = z - MOUNT_HEIGHT
        
        cos_theta3 = (r**2 + z_eff**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(cos_theta3) > 1.0 :
            response.success = False
            response.message = "Target is out of reach"
            return response  # target is out of reach
        
        forearm_angle = math.atan2(math.sqrt(1 - cos_theta3**2), cos_theta3)
        upper_arm_angle = math.atan2(z_eff, r) - math.atan2(L2 * math.sin(forearm_angle), L1 + L2 * cos_theta3)
        wrist_angle = 0.0  # keep gripper level

        # publish joint angles
        self.publisher1.publish(Float64(data=base_angle))
        self.publisher2.publish(Float64(data=upper_arm_angle))
        self.publisher3.publish(Float64(data=forearm_angle))
        self.publisher4.publish(Float64(data=wrist_angle))
        
        response.joint1 = base_angle
        response.joint2 = upper_arm_angle
        response.joint3 = forearm_angle
        response.joint4 = wrist_angle
        response.message = "IK solution found"
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    arm_ik_node = ArmIK()
    rclpy.spin(arm_ik_node)
    arm_ik_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
