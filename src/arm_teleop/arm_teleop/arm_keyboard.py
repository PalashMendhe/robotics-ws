#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import tty
import termios

STEP = 0.05  # radians per keypress
GRIPPER_STEP = 0.005  # meters per keypress

KEYS = {
    'w': ('upper_arm_joint', +1),
    's': ('upper_arm_joint', -1),
    'a': ('arm_base_joint', +1),
    'd': ('arm_base_joint', -1),
    'r': ('forearm_joint', +1),
    'f': ('forearm_joint', -1),
    't': ('wrist_joint', +1),
    'g': ('wrist_joint', -1),
    'o': ('gripper', +1),   # open
    'c': ('gripper', -1),   # close
}

LIMITS = {
    'arm_base_joint':  (-3.14, 3.14),
    'upper_arm_joint': (-1.5707, 1.5707),
    'forearm_joint':   (-1.5707, 1.5707),
    'wrist_joint':     (-3.14, 3.14),
}

class ArmKeyboard(Node):
    def __init__(self):
        super().__init__('arm_keyboard')
        self.pubs = {
            'arm_base_joint':  self.create_publisher(Float64, '/arm_base_joint/cmd_pos', 10),
            'upper_arm_joint': self.create_publisher(Float64, '/upper_arm_joint/cmd_pos', 10),
            'forearm_joint':   self.create_publisher(Float64, '/forearm_joint/cmd_pos', 10),
            'wrist_joint':     self.create_publisher(Float64, '/wrist_joint/cmd_pos', 10),
            'left_prong':      self.create_publisher(Float64, '/left_prong_joint/cmd_pos', 10),
            'right_prong':     self.create_publisher(Float64, '/right_prong_joint/cmd_pos', 10),
        }
        self.positions = {
            'arm_base_joint':  0.0,
            'upper_arm_joint': 0.0,
            'forearm_joint':   0.0,
            'wrist_joint':     0.0,
            'gripper':         0.0,
        }
        self.print_help()

    def print_help(self):
        print("""
ARM KEYBOARD CONTROLLER
-----------------------
w/s : upper arm up/down
a/d : base rotate left/right
r/f : forearm up/down
t/g : wrist roll
o/c : gripper open/close
q   : quit
""")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def clamp(self, joint, value):
        lo, hi = LIMITS.get(joint, (-3.14, 3.14))
        return max(lo, min(hi, value))

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == 'q':
                break
            if key not in KEYS:
                continue
            joint, direction = KEYS[key]
            if joint == 'gripper':
                self.positions['gripper'] += direction * GRIPPER_STEP
                self.positions['gripper'] = max(0.0, min(0.08, self.positions['gripper']))
                msg = Float64()
                msg.data = self.positions['gripper']
                self.pubs['right_prong'].publish(msg)
                msg2 = Float64()
                msg2.data = -self.positions['gripper']
                self.pubs['left_prong'].publish(msg2)
                print(f"gripper: {self.positions['gripper']:.3f}")
            else:
                self.positions[joint] += direction * STEP
                self.positions[joint] = self.clamp(joint, self.positions[joint])
                msg = Float64()
                msg.data = self.positions[joint]
                self.pubs[joint].publish(msg)
                print(f"{joint}: {self.positions[joint]:.3f}")

def main():
    rclpy.init()
    node = ArmKeyboard()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()