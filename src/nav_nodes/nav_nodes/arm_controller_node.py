#!/usr/bin/env python3
"""
arm_controller_node.py
──────────────────────
Executes the pick-and-place sequence for arm1 (shelf_1 box_1 → docked-bot tray)
driven by the ros2_control joint_trajectory controllers in the CURRENT
multiroom world:

  arm1  spawn : (0.75, 0.30, 0.40),  yaw 0   (arm_gazebo.launch.py)
  box_1 centre: (0.30, 0.30, 0.46)            → rel (-0.45,  0.00,  0.05)
  bot tray    : docked ~(0.335, 0.7431, 0.14) → rel (-0.415, 0.44, -0.27)

Why no MoveIt / planning_scene_manager?
---------------------------------------
This workspace does NOT run a MoveGroup server (only `moveit_msgs` message
types are present — no moveit_ros_planning / move_group node is launched, and
`planning_scene_manager` / `moveit_commander` are not installed). The ActionClient
against `/move_action` would block forever on `wait_for_server()`. The arm and
gripper here are commanded directly through the namespaced ros2_control
controller topics that ARE running:

  /<arm_ns>/arm_controller/joint_trajectory
  /<arm_ns>/gripper_controller/joint_trajectory

Collision-awareness is approximated instead by a conservative, fixed joint
sequence kept clear of the shelf, walls and the docked bot (see waypoints).

Sequence
--------
  IDLE
  → home        (safe upright configuration)
  → open grip   (gripper fully open)
  → prep        (above box_1, clear of the shelf)
  → grasp       (reach down to the box centre)
  → grip        (close the prongs around the box)
  → carry       (lift to safe height)
  → transport   (swing over the docked bot tray)
  → lower       (descend into the tray)
  → release     (open gripper, drop the box)
  → retract     (rise clear of the tray)
  → home        (return upright)
  → DONE

Trigger / result protocol (used by docking_node)
------------------------------------------------
  Subscribes to  /<arm_ns>/trigger            (std_msgs/Bool)
  Publishes done to /<arm_ns>/done            (std_msgs/Bool, TRANSIENT_LOCAL/latched)

Dependencies (all available in the current workspace):
  rclpy, std_msgs, builtin_interfaces, trajectory_msgs
"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ── Arm geometry (UR5 DH, matches arm.urdf.xacro) ───────────────────────────
_D4      = 0.109    # wrist-1 z offset
_L1      = 0.425    # upper arm length  (|a2|)
_L2      = 0.392    # forearm length    (|a3|)
_TCP_OFF = 0.170    # gripper_base_link → gripper_tcp (0.075 + prong geometry)

# Joints:  arm_base, upper_arm, forearm, wrist, gripper_baseTOwrist, gripper_base
ARM_JOINTS = [
    'arm_base_joint',
    'upper_arm_joint',
    'forearm_joint',
    'wrist_joint',
    'gripper_baseTOwrist_joint',
    'gripper_base_joint',
]

# Prismatic gripper prongs (limits: right 0→0.08, left -0.08→0)
GRIPPER_JOINTS = ['right_prong_joint', 'left_prong_joint']
GRIPPER_OPEN   = [0.04, -0.04]     # fully open
GRIPPER_CLOSED = [0.028, -0.028]   # grip the 0.15 m box_1


def _wrap_pi(a: float) -> float:
    """Wrap an angle into (-pi, pi] (arm_base/wrist limits are ±pi)."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _ik(x: float, y: float, z: float):
    """
    Analytic IK for the 4-DoF planar arm (shoulder pan, lift, elbow, wrist).

    x, y, z are the TCP target in the arm-local frame (arm base at origin).
    Mirrors the well-tested solver of the legacy node; the wrist-roll joints
    are parked at 0 (gripper points straight down).
    """
    r = math.sqrt(x * x + y * y)
    theta1 = _wrap_pi(math.atan2(y, x) + math.asin(_D4 / r))
    r_planar = math.sqrt(r * r - _D4 * _D4)

    dz = z + _TCP_OFF - 0.02
    d = math.sqrt(r_planar * r_planar + dz * dz)
    beta = math.atan2(dz, r_planar)
    cos_gamma = (_L1 ** 2 + d ** 2 - _L2 ** 2) / (2.0 * _L1 * d)
    gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))
    theta2 = math.pi - (beta + gamma)

    cos_theta3 = (d ** 2 - _L1 ** 2 - _L2 ** 2) / (2.0 * _L1 * _L2)
    theta3 = math.acos(max(-1.0, min(1.0, cos_theta3)))
    theta4 = _wrap_pi(theta2 + theta3 + math.pi / 2.0)

    return [theta1, theta2, theta3, theta4, 0.0, 0.0]


# ── Current-world waypoints (arm1 base frame ≈ world (0.75, 0.30, 0.41)) ────
# Shelf-1 pick side (box_1 at world (0.30, 0.30, 0.46)) — reach toward -X.
PICK_PREP   = _ik(-0.45,  0.00,  0.18)    # above the box, clear of the shelf
PICK_GRASP  = _ik(-0.45,  0.00,  0.05)    # TCP at box centre
CARRY       = _ik(-0.45,  0.00,  0.28)    # lift the box above the shelf

# Bot-tray side (bot docks ~0.307 m off arcuo_dock_shelf1 at (0.025, 0.7431),
# so its tray centre lands ≈ (0.335, 0.7431, 0.14)) — reach toward -X/+Y, low.
PLACE_OVER  = _ik(-0.415, 0.44, -0.12)    # above the tray
PLACE_DROP  = _ik(-0.415, 0.44, -0.27)    # TCP down into the tray
RETRACT     = _ik(-0.30,  0.35, -0.10)    # rise clear of the tray

# Safe upright parked stance (shoulder folded, wrist tucked).
HOME = [0.0, -1.5708, 0.0, -1.5708, -1.5708, 0.0]


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        # Namespace of the arm this node serves (arm1 / arm2). Used for the
        # controller trajectory topics AND the trigger/done protocol.
        self.declare_parameter('arm_ns', 'arm1')
        ns = f"/{self.get_parameter('arm_ns').value}"

        # ── publishers: ros2_control JointTrajectory controllers ─────────────
        self.arm_pub = self.create_publisher(
            JointTrajectory, f'{ns}/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, f'{ns}/gripper_controller/joint_trajectory', 10)

        # TRANSIENT_LOCAL: late-joining subscribers (e.g. docking_node) still
        # receive the last 'done'.
        done_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(Bool, f'{ns}/done', done_qos)

        # ── trigger subscriber (fired by docking_node after docking) ─────────
        # TRANSIENT_LOCAL: pairing with docking_node's latched trigger publisher.
        # If THIS node starts after the dock already fired+latched trigger, the very
        # last value replays the instant we subscribe — otherwise we'd miss the only
        # one-shot trigger and deadlock (dock waiting on /done).

        trig_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Bool, f'{ns}/trigger', self._trigger_cb, trig_qos)

        self._timer = None
        self._running = False
        self.get_logger().info(f'arm_controller_node ready — waiting on {ns}/trigger')

    # ── trigger callback ──────────────────────────────────────────────────────
    def _trigger_cb(self, msg: Bool):
        if not msg.data or self._running:
            return
        self._running = True
        self.get_logger().info('Trigger received → starting pick-and-place')
        self._run_sequence()

    # ── main sequence (one-shot rclpy timer chain) ────────────────────────────
    def _run_sequence(self):
        # NOTE: rclpy timers REPEAT by default — the handle is stored so each
        # step can cancel/destroy it and become a true one-shot (see bug.md).
        self._steps = [
            ('HOME    — safe upright',       lambda: self._move_arm(HOME,        secs=2)),
            ('OPEN    — gripper open',       lambda: self._move_grip(GRIPPER_OPEN,  secs=1)),
            ('PREP    — above box_1',        lambda: self._move_arm(PICK_PREP,   secs=3)),
            ('GRASP   — reach to box',       lambda: self._move_arm(PICK_GRASP,  secs=3)),
            ('GRIP    — close gripper',      lambda: self._move_grip(GRIPPER_CLOSED, secs=1)),
            ('CARRY   — lift box',           lambda: self._move_arm(CARRY,       secs=2)),
            ('TRANSPORT — swing to tray',    lambda: self._move_arm(PLACE_OVER,  secs=3)),
            ('LOWER   — descend into tray',  lambda: self._move_arm(PLACE_DROP,  secs=2)),
            ('RELEASE — open gripper',       lambda: self._move_grip(GRIPPER_OPEN,  secs=1)),
            ('RETRACT — rise clear',         lambda: self._move_arm(RETRACT,     secs=2)),
            ('HOME    — return upright',     lambda: self._move_arm(HOME,        secs=2)),
        ]
        self._step_idx = 0
        self._timer = None
        self._schedule_next(1.0)

    def _schedule_next(self, delay_secs):
        self._timer = self.create_timer(delay_secs, self._execute_step)

    def _execute_step(self):
        # One-shot: cancel/destroy the timer that just fired, otherwise it
        # keeps repeating and stacks timers on every step (see bug.md).
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        if self._step_idx >= len(self._steps):
            self.get_logger().info('Sequence DONE')
            done = Bool(); done.data = True
            self.done_pub.publish(done)
            self._running = False
            return

        label, fn = self._steps[self._step_idx]
        self.get_logger().info(f'Step {self._step_idx + 1}/{len(self._steps)}: {label}')
        duration_secs = fn()                     # publish the trajectory
        self._step_idx += 1
        self._schedule_next(duration_secs + 0.5) # motion time + 0.5 s buffer

    # ── trajectory helpers ────────────────────────────────────────────────────
    def _move_arm(self, positions, secs=2):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        pt.time_from_start = Duration(sec=secs, nanosec=0)
        msg.points = [pt]
        self.arm_pub.publish(msg)
        return secs

    def _move_grip(self, positions, secs=1):
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = Duration(sec=secs, nanosec=0)
        msg.points = [pt]
        self.grip_pub.publish(msg)
        return secs


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()