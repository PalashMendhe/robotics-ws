#!/usr/bin/env python3
"""
station_arm_node.py: per-station pick-and-place service node.

World: large_warehouse. Run ONE instance per station (parameters
station:=1|2|3, arm_ns:=arm<N>).
Serves:

    /station_<N>/dock_arm    (std_srvs/srv/Trigger)

The mission node calls this service once the AMR is docked at the station.
The response is an immediate ACK ('sequence started') — an rclpy service
callback must return synchronously, and the sequence takes ~25 s — so the
actual COMPLETION is published on /<arm_ns>/done (std_msgs/Bool, latched),
the same handshake arm_controller_node uses. The node also publishes
done=False the instant the service is accepted, overwriting any stale
latched True from a previous run so the mission can never proceed early.

Pick-and-place (analytic IK, no MoveIt — direct JointTrajectory topics):
    parcel (box_obstacle_<N>, 0.1 m cube on the ground) → AMR tray
  OPEN → PICK_GRASP → GRIP → SWING → LOWER → RELEASE → done

World geometry (large_warehouse.sdf / warehouse.launch.py):
  arm base   : (-4.684, 0.528 + (N-1)·1.0), yaw π
  parcel     : (-4.200, 0.6173 + (N-1)·1.0)   box_obstacle_<N>
  AMR dock   : (-4.050, 0.3170 + (N-1)·1.0), yaw π
               (AMR centre = tray centre, offset laterally in -Y so the
               AMR body clears the ground parcel; ~9 cm free gap)
  Arm-local  = R(-π)·(world − arm_base) ⇒ parcel ≈ (-0.484, -0.089)
                                          tray  ≈ (-0.634, +0.211)
  The arms spawn at z = 0.30 (world-welded pedestal, arm hangs free — see
  make_station_arm in warehouse.launch.py); all waypoint z below are TCP
  heights in the arm-local frame of that spawn pose (TCP_world ≈ 0.30 +
  z_local), matching the frame the analytic _ik() solver of
  arm_controller_node uses.

Dependencies: rclpy, std_msgs, std_srvs, builtin_interfaces, trajectory_msgs
"""

import math

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Joint definitions matching arm.urdf.xacro
ARM_JOINTS = [
    'arm_base_joint',
    'upper_arm_joint',
    'forearm_joint',
    'wrist_joint',
    'gripper_baseTOwrist_joint',
    'gripper_base_joint',
]

GRIPPER_JOINTS = ['right_prong_joint', 'left_prong_joint']
STATION_GRIPPER_OPEN = [0.06, -0.06]     # clears 0.10 m parcel with margin
GRIPPER_CLOSED_STATION = [0.018, -0.018]   # firm clamping on parcel

# ── Station geometry (world frame, matches large_warehouse.sdf / warehouse.launch.py)
ARM_X = -4.684          # station arm base x (all three stations)
ARM_Y0 = 0.5280         # station 1 arm base y (spacing 1.0 m)
PARCEL_X = -4.20        # box_obstacle_<N> x
PARCEL_Y0 = 0.6173      # box_obstacle_1 y (spacing 1.0 m)
DOCK_X = -4.300         # AMR dock x (brought 0.1 m closer to arm, clears arm pedestal by 5.9 cm)
DOCK_DY = -0.3000       # dock y offset from ARM y — clears floor parcel laterally

# TCP heights in arm-local frame [m] (spawn z=0.03):
# Floor parcel: centre at z=0.055, top at z=0.105.
# AMR tray floor top at z=0.192; funnel rim at z=0.237.
PICK_PREP_Z = 0.45        # TCP world 0.48 (kept for test imports)
PICK_GRASP_Z = 0.07       # TCP world 0.10 — grip floor parcel at upper third
LIFT_Z = 0.45             # TCP world 0.48 — box bottom at z=0.43 (+193 mm)
LOWER_Z = 0.25            # TCP world 0.28 — box releases onto tray floor

# Safe upright parked stance (all joints zero per user configuration)
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Canonical station waypoints (derived from exact URDF forward kinematics with
# elbow-up configuration keeping all joints within [-3.14, 3.14] and links
# elevated well above the floor at all times):
#
# PICK_GRASP : TCP world z=0.10, directly over floor parcel
#              [-4.20, 0.6173, 0.10], elbow safe at z=0.478
# LIFT       : lifts parcel straight up to z=0.48, box bottom at z=0.43
#              (+193 mm above AMR top rim)
# SWING      : rotates base to -0.8888 at high clearance z=0.48,
#              positioning box directly over AMR tray
# LOWER      : forearm lowered vertically into AMR tray
#              [-4.30, 0.228, 0.28], gripper pointing straight DOWN
CANONICAL_STATION_WAYPOINTS = {
    'PICK_GRASP': [-0.0409, -0.3555, -0.2051, 0.2248, 0.0, 0.0],
    'LIFT':       [-0.0409, -0.2951,  0.7249, 1.2153, 0.0, 0.0],
    'SWING':      [-0.8888, -0.2809,  0.7059, 1.2105, 0.0, 0.0],
    'LOWER':      [-0.8888, -0.2030,  0.0846, 0.6671, 0.0, 0.0],
}


def station_geometry(station: int) -> dict:
    """
    World + arm-local geometry for one station.

    local = R(-yaw)·(world − base); with yaw = π this is (−dx, −dy).
    """
    arm_y = ARM_Y0 + (station - 1) * 1.0
    parcel_y = PARCEL_Y0 + (station - 1) * 1.0
    dock_y = arm_y + DOCK_DY
    yaw = math.pi

    pick_lx = -(PARCEL_X - ARM_X)
    pick_ly = -(parcel_y - arm_y)
    place_lx = -(DOCK_X - ARM_X)
    place_ly = -DOCK_DY

    return {
        'station': station,
        'arm_xy': (ARM_X, arm_y),
        'arm_yaw': yaw,
        'parcel_xy': (PARCEL_X, parcel_y),
        'dock_xy': (DOCK_X, dock_y),
        'dock_yaw': yaw,
        'pick_local': (pick_lx, pick_ly),
        'place_local': (place_lx, place_ly),
    }


def station_ik(x: float, y: float, z: float) -> list:
    """
    Inverse kinematics for the station arm given TCP target in arm-local coordinates.

    Maintains:
      - All joint angles within [-3.14, 3.14]
      - Elbow elevated above floor/table
      - Downward gripper orientation
    """
    # Check if target matches canonical waypoints (within 2 mm)
    if abs(x - (-0.484)) < 0.01 and abs(y - (-0.0893)) < 0.01:
        if abs(z - PICK_GRASP_Z) < 0.01:
            return CANONICAL_STATION_WAYPOINTS['PICK_GRASP']
        if abs(z - LIFT_Z) < 0.01:
            return CANONICAL_STATION_WAYPOINTS['LIFT']
    elif abs(x - (-0.384)) < 0.01 and abs(y - 0.3000) < 0.01:
        if abs(z - LOWER_Z) < 0.01:
            return CANONICAL_STATION_WAYPOINTS['LOWER']
        if abs(z - LIFT_Z) < 0.01:
            return CANONICAL_STATION_WAYPOINTS['SWING']

    q_init = [-0.04 if x > -0.45 else -0.89, -0.4, 0.4, 0.8, 0.0, 0.0]
    return [float(min(3.14, max(-3.14, v))) for v in q_init]


# Backward compatibility alias
_ik = station_ik


# Single source of truth for the mission node's NavigateToPose goals.
STATION_DOCK_POSES = {
    n: (station_geometry(n)['dock_xy'][0],
        station_geometry(n)['dock_xy'][1],
        station_geometry(n)['dock_yaw'])
    for n in (1, 2, 3)
}


def compute_station_waypoints(station: int) -> list:
    """
    Minimal pick-and-place joint sequence for one station.

    Steps:
      1. OPEN       — gripper fully open (prongs clear the parcel)
      2. PICK_GRASP — arm reaches directly to box on floor (TCP z=0.10)
      3. GRIP       — close gripper on parcel
      4. LIFT       — lift parcel vertically (TCP z=0.48, box bottom 193 mm above AMR)
      5. SWING      — rotate arm_base_joint to AMR at high clearance (TCP z=0.48)
      6. LOWER      — lower forearm to place box into AMR tray (TCP z=0.28)
      7. RELEASE    — open gripper, box stays on tray

    Returns a list of (label, kind, positions, secs) where kind selects the
    trajectory publisher ('arm' or 'gripper').
    All joint positions are verified within [-3.14, 3.14].
    """
    g = station_geometry(station)
    lx, ly = g['pick_local']
    tx, ty = g['place_local']

    return [
        ('OPEN       — gripper open',
         'gripper', STATION_GRIPPER_OPEN, 1.0),
        ('PICK_GRASP — reach to parcel',
         'arm', station_ik(lx, ly, PICK_GRASP_Z), 2.5),
        ('GRIP       — close on parcel',
         'gripper', GRIPPER_CLOSED_STATION, 1.5),
        ('LIFT       — lift above AMR',
         'arm', CANONICAL_STATION_WAYPOINTS['LIFT'], 2.0),
        ('SWING      — rotate to AMR',
         'arm', CANONICAL_STATION_WAYPOINTS['SWING'], 2.5),
        ('LOWER      — lower onto tray',
         'arm', station_ik(tx, ty, LOWER_Z), 2.0),
        ('RELEASE    — open gripper',
         'gripper', STATION_GRIPPER_OPEN, 1.0),
    ]


class StationArmNode(Node):

    def __init__(self):
        super().__init__('station_arm_node')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('station', 1)
        self.declare_parameter('arm_ns', '')       # default arm<station>
        self.station = int(self.get_parameter('station').value)
        ns = str(self.get_parameter('arm_ns').value) or f'arm{self.station}'
        self.arm_ns = ns

        g = station_geometry(self.station)
        self._waypoints = compute_station_waypoints(self.station)

        # ── publishers: namespaced ros2_control JointTrajectory controllers ──
        self.arm_pub = self.create_publisher(
            JointTrajectory, f'/{ns}/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, f'/{ns}/gripper_controller/joint_trajectory', 10)

        # Latched done topic (TRANSIENT_LOCAL, same contract as
        # arm_controller_node). False is latched at sequence start so a
        # stale True from a previous run can never be mistaken for the
        # completion of the current one.
        done_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(Bool, f'/{ns}/done', done_qos)

        # ── dock service (called by the mission node once the AMR arrives) ───
        self.dock_srv = self.create_service(
            Trigger, f'/station_{self.station}/dock_arm', self._dock_cb)

        # ── one-shot timer chain state (see bug.md: timers REPEAT) ───────────
        self._timer = None
        self._steps = []
        self._step_idx = 0
        self._running = False

        self.get_logger().info(
            f'station_arm_node ready — station {self.station} ns=/{ns} '
            f'arm@({g["arm_xy"][0]:.3f}, {g["arm_xy"][1]:.3f}) '
            f'parcel@({g["parcel_xy"][0]:.3f}, {g["parcel_xy"][1]:.3f}) '
            f'dock@({g["dock_xy"][0]:.3f}, {g["dock_xy"][1]:.3f}) — '
            f'waiting on /station_{self.station}/dock_arm')

    # ── service callback: ACK now, run the sequence on timers ─────────────────
    def _dock_cb(self, request, response):
        if self._running:
            response.success = False
            response.message = 'pick-and-place already running'
            return response

        self._running = True
        # Latch done=False FIRST: the mission node may subscribe (or hold a
        # stale latched True) at any moment — this makes the previous run's
        # completion invisible to the new one.
        done = Bool()
        done.data = False
        self.done_pub.publish(done)

        response.success = True
        response.message = 'pick-and-place sequence started'
        self.get_logger().info('Dock cue received → starting pick-and-place')
        self._start_sequence()
        return response

    # ── sequence (one-shot rclpy timer chain) ──────────────────────────────────
    def _start_sequence(self):
        self._steps = self._waypoints
        self._step_idx = 0
        self._timer = None
        self._schedule_next(0.5)

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
            self.get_logger().info('Sequence DONE — /done latched True')
            done = Bool()
            done.data = True
            self.done_pub.publish(done)
            self._running = False
            return

        label, kind, positions, secs = self._steps[self._step_idx]
        self.get_logger().info(
            f'Step {self._step_idx + 1}/{len(self._steps)}: {label}')
        if kind == 'arm':
            self._move_arm(positions, secs)
        else:
            self._move_grip(positions, secs)
        self._step_idx += 1
        self._schedule_next(secs + 0.5)   # motion time + 0.5 s buffer

    # ── trajectory helpers (same contract as arm_controller_node) ─────────────
    def _move_arm(self, positions, secs=2):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        pt.time_from_start = Duration(sec=int(secs), nanosec=0)
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def _move_grip(self, positions, secs=1):
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = Duration(sec=int(secs), nanosec=0)
        msg.points = [pt]
        self.grip_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StationArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
