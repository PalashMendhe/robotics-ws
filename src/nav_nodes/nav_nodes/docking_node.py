#!/usr/bin/env python3
"""
docking_node.py
───────────────
Vision-guided reverse docking for the multiroom pick-and-place cell.

When run, the bot drives to the configured pre-dock waypoint, then uses the
ArUco marker TF published by aruco_detector_node to reverse-dock: the marker
is tracked on the REAR camera while the bot backs up, nulling lateral offset
and heading error, until the camera→marker range reaches the calibrated dock
distance. Docking is then latched, /<arm_ns>/trigger is published, and the
node waits for /<arm_ns>/done from arm_controller_node.

State machine
-------------
  DRIVE   — odom servo to the pre-dock waypoint (x, y), then align yaw
  SERVO   — TF-based reverse docking (range + bearing + heading control)
  LATCH   — range stable at target → stop, zero cmd_vel, publish trigger
  WAIT    — /<arm_ns>/done received → DONE (bot stays put)

Range convention
----------------
Range is ‖tvec‖ = camera_frame → marker_frame, i.e. the exact metric the
aruco_detector_node logs (and the one the 0.307 m threshold was measured
from). Control errors (bearing, heading) use marker pose in base_link.

Parameters
  mode            'park'    'park' = drive to the wait pose, stop, trigger arm
                            'marker' = additionally reverse-dock on the marker
                            TF until the camera→marker range reaches
                            target_range (0.307 m) before triggering
  arm_ns          'arm1'    trigger/done topic namespace
  marker_id       0         ArUco ID of this dock's marker
  camera_frame    'camera_link_2'   rear camera optical frame
  base_frame      'base_link'
  wait_x/y/yaw    0.5 / 0.8 / 0.0   WORLD-frame pose where the bot parks
  spawn_x/y/yaw   2.2 / 0.8 / 0.0   bot's Gazebo spawn pose (gazebo.launch.py).
                                    Gazebo odometry starts at ZERO at this
                                    pose, so world targets must be shifted
                                    into the odom frame before driving.
  target_range    0.307             dock distance, camera → marker [m]
                                    (marker mode only)
  Shelf 2 / arm 2: arm_ns=arm2, marker_id=1, wait_x=6.7, wait_y=5.4463,
  wait_yaw=3.14159.

NOTE: only ONE node may publish /model/my_robot/cmd_vel. If the YOLO
state_machine or a waypoint navigator is running, the bot receives
conflicting commands and appears to wander randomly.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def _wrap(a):
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_of(q):
    """Yaw from a quaternion (ignores roll/pitch — valid for a planar bot)."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockingNode(Node):

    DRIVE, SERVO, LATCH, WAIT, DONE = range(5)
    _STATE_NAMES = ['DRIVE', 'SERVO', 'LATCH', 'WAIT', 'DONE']

    def __init__(self):
        super().__init__('docking_node')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('mode',          'park')   # 'park' | 'marker'
        self.declare_parameter('arm_ns',        'arm1')
        self.declare_parameter('marker_id',     0)
        self.declare_parameter('camera_frame',  'camera_link_2')
        self.declare_parameter('base_frame',    'base_link')
        self.declare_parameter('wait_x',        0.5)
        self.declare_parameter('wait_y',        0.8)
        self.declare_parameter('wait_yaw',      0.0)
        self.declare_parameter('spawn_x',       2.2)
        self.declare_parameter('spawn_y',       0.8)
        self.declare_parameter('spawn_yaw',     0.0)
        self.declare_parameter('target_range',  0.307)

        p = lambda n: self.get_parameter(n).value
        self.mode          = str(p('mode'))
        self.arm_ns        = p('arm_ns')
        self.marker_frame  = f"aruco_marker_{p('marker_id')}_rear"
        self.camera_frame  = p('camera_frame')
        self.base_frame    = p('base_frame')
        self.wait_pose     = (p('wait_x'), p('wait_y'), p('wait_yaw'))
        self.target_range  = p('target_range')

        # Convert the WORLD-frame wait pose into the odom frame. Gazebo's
        # diff-drive odometry starts at (0, 0, 0) at the bot's SPAWN pose, so
        # odom = R(-spawn_yaw) · (world - spawn_xy), yaw_odom = yaw - spawn_yaw.
        sx, sy, syaw = p('spawn_x'), p('spawn_y'), p('spawn_yaw')
        dxw, dyw = self.wait_pose[0] - sx, self.wait_pose[1] - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        self.wait_odom = (c * dxw - s * dyw,
                          s * dxw + c * dyw,
                          _wrap(self.wait_pose[2] - syaw))

        # ── control gains / limits ────────────────────────────────────────────
        self.max_vx      = 0.08    # m/s
        self.max_wz      = 0.4     # rad/s
        self.k_drive     = 0.8     # distance gain (go-to-goal)
        self.k_w         = 1.0     # heading gain (go-to-goal)
        self.k_v         = 0.8     # range servo (marker mode)
        self.k_yaw       = 1.5     # heading servo (marker mode)
        self.k_align     = 1.2     # pre-dock yaw alignment
        self.range_tol   = 0.005   # [m] latch tolerance
        self.overshoot   = 0.02    # [m] past-target emergency stop
        self.yaw_gate    = 0.30    # [rad] only back up when roughly aligned
        self.debounced   = 0.5     # [s] range must hold at target this long
        self.tf_lost_to  = 3.0     # [s] marker TF loss tolerated in SERVO

        # ── interfaces ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/model/my_robot/cmd_vel', 10)

        # TRANSIENT_LOCAL (latched): a late-starting arm_controller_node (e.g.
        # started AFTER the bot already docked) still receives the very last trigger
        # value the instant it subscribes. Without this, the one-shot VOLATILE trigger
        # is gone by the time the arm node subscribes, and both nodes deadlock —
        # dock waits on /done, arm waits forever on /trigger.
        trigger_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trig_pub = self.create_publisher(Bool, f'/{self.arm_ns}/trigger', trigger_qos)

        done_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, f'/{self.arm_ns}/done', self._done_cb, done_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── state ─────────────────────────────────────────────────────────────
        self.state        = self.DRIVE
        self.stable_since = None   # settle/debounce timer
        self.last_seen    = None   # last time marker TF was available (SERVO)
        self._last_trig   = None   # last time the trigger was (re)published (WAIT)

        # Safety: another cmd_vel publisher = tug-of-war = wandering bot
        n_pub = self.count_publishers('/model/my_robot/cmd_vel')
        if n_pub > 1:
            self.get_logger().error(
                f'{n_pub} publishers on /model/my_robot/cmd_vel — ANOTHER '
                'controller (state_machine? waypoint_navigator? teleop?) will '
                'fight this node and the bot will wander. Stop it first.')

        self.create_timer(0.05, self._control_loop)   # 20 Hz
        self.get_logger().info(
            f'docking_node ready — mode={self.mode} dock={self.arm_ns} '
            f'wait(world)=({self.wait_pose[0]:.2f}, {self.wait_pose[1]:.2f}, '
            f'{self.wait_pose[2]:.2f} rad) → odom target='
            f'({self.wait_odom[0]:.2f}, {self.wait_odom[1]:.2f}, '
            f'{self.wait_odom[2]:.2f} rad) marker={self.marker_frame}')

    # ── helpers ───────────────────────────────────────────────────────────────
    def _look(self, target, source):
        """Latest TF lookup (time=0 → immune to sim/wall clock mismatch)."""
        try:
            return self.tf_buffer.lookup_transform(target, source, Time())
        except Exception:
            return None

    def _bot_pose(self):
        """Bot pose in odom as (x, y, yaw) or None."""
        tf = self._look('odom', self.base_frame)
        if tf is None:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, _yaw_of(q)

    def _send(self, vx, wz):
        cmd = Twist()
        cmd.linear.x = float(max(-self.max_vx, min(self.max_vx, vx)))
        cmd.angular.z = float(max(-self.max_wz, min(self.max_wz, wz)))
        self.cmd_pub.publish(cmd)

    def _set_state(self, s, msg=''):
        self.state = s
        self.get_logger().info(f'→ state {self._STATE_NAMES[s]} {msg}')

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _done_cb(self, msg):
        if msg.data and self.state == self.WAIT:
            self._set_state(self.DONE, '— arm sequence complete, bot holding position')

    # ── main loop ─────────────────────────────────────────────────────────────
    def _control_loop(self):
        if self.state == self.DRIVE:
            self._drive()
        elif self.state == self.SERVO:
            self._servo()
        elif self.state in (self.LATCH, self.WAIT):
            self._hold()

    # ── DRIVE: go-to-goal to the wait pose, settle, then latch/servo ──────────
    #
    # NOTE: this sim robot's 4-fixed-wheel skid-steer cannot pivot reliably
    # in place (lateral slip → chassis wanders while wheel odom lies). So the
    # controller NEVER "turn first, then drive": it arcs toward the target,
    # and when the target is behind it REVERSES straight back — no rotation.
    def _drive(self):
        pose = self._bot_pose()
        if pose is None:
            return   # no odom TF yet — hold
        bx, by, byaw = pose
        tx, ty, tyaw = self.wait_odom

        dx, dy = tx - bx, ty - by
        dist = math.hypot(dx, dy)

        if dist > 0.03:
            self.stable_since = None
            err = _wrap(math.atan2(dy, dx) - byaw)   # heading error to target
            if abs(err) <= math.pi / 2.0:
                # target in front — drive forward, arc on heading error
                vx = self.k_drive * dist * math.cos(err)
                wz = self.k_w * math.sin(err)
            else:
                # target behind — back straight toward it (no in-place turn);
                # steering reverses when reversing.
                vx = -self.k_drive * dist * abs(math.cos(err))
                wz = -self.k_w * math.sin(err)
            self._send(vx, wz)
            return

        # at the pose — align the wait yaw
        yaw_err = _wrap(tyaw - byaw)
        if abs(yaw_err) > 0.05:
            self.stable_since = None
            self._send(0.0, self.k_align * yaw_err)
            return

        # settled — hold the pose for the debounce window before proceeding
        self._send(0.0, 0.0)
        now = self.get_clock().now()
        if self.stable_since is None:
            self.stable_since = now
            self.get_logger().info(
                f'At wait pose ({tx:.2f}, {ty:.2f}) — settling')
        if (now - self.stable_since).nanoseconds / 1e9 < self.debounced:
            return
        self.stable_since = None

        if self.mode == 'marker':
            self._set_state(self.SERVO, '— parked, marker servo next')
        else:
            self._set_state(
                self.LATCH,
                f'— parked at world ({self.wait_pose[0]:.2f}, '
                f'{self.wait_pose[1]:.2f}, {self.wait_pose[2]:.2f})')

    # ── HOLD: freeze bot; fire trigger on entry; guard position in WAIT ───────
    def _hold(self):
        if self.state == self.LATCH:
            self._latch()          # publishes trigger once, switches to WAIT
        self._send(0.0, 0.0)       # keep winning any cmd_vel fight
        if self.state == self.WAIT:
            pose = self._bot_pose()
            if pose and math.hypot(pose[0] - self.wait_odom[0],
                                   pose[1] - self.wait_odom[1]) > 0.05:
                self.get_logger().warn(
                    'Bot displaced >5 cm from wait pose — is another '
                    'cmd_vel publisher running?',
                    throttle_duration_sec=5.0)

            # Re-announce the trigger every ~1 s while waiting. A latched
            # (TRANSIENT_LOCAL) sample is only delivered to subscribers that were
            # already matched when it was published — an arm_controller_node that
            # (re)starts after the dock will still replay it via the latched value, but
            # re-publishing makes the handshake robust against any RMW / QoS quirk
            # and covers the case where the arm completed and was restarted before
            # the done round-trip finished.
            now = self.get_clock().now()
            if self._last_trig is None or \
                    (now - self._last_trig).nanoseconds / 1e9 >= 1.0:
                self._last_trig = now
                trig = Bool(); trig.data = True
                self.trig_pub.publish(trig)

    # ── SERVO: TF-based reverse docking ───────────────────────────────────────
    def _servo(self):
        now = self.get_clock().now()

        cam_tf = self._look(self.camera_frame, self.marker_frame)
        base_tf = self._look(self.base_frame, self.marker_frame)
        if cam_tf is None or base_tf is None:
            # marker not in view — hold position, but don't wait forever
            if self.last_seen is not None and \
                    (now - self.last_seen).nanoseconds / 1e9 > self.tf_lost_to:
                self.get_logger().error(
                    'Marker TF lost >3 s in SERVO — holding position. '
                    'Recover visibility or restart the node.',
                    throttle_duration_sec=5.0)
            self._send(0.0, 0.0)
            return
        self.last_seen = now

        # range: camera → marker (the detector-log metric, matches 0.307 cal)
        t = cam_tf.transform.translation
        rng = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)

        # heading error: marker should sit dead astern (azimuth = pi)
        b = base_tf.transform.translation
        azimuth = math.atan2(b.y, b.x)          # marker bearing in base frame
        yaw_err = _wrap(azimuth - math.pi)      # 0 when marker dead astern

        # latch: range at target, held stable for the debounce window
        if rng <= self.target_range + self.range_tol:
            if self.stable_since is None:
                self.stable_since = now
            held = (now - self.stable_since).nanoseconds / 1e9
            self._send(0.0, 0.0)
            if held >= self.debounced:
                self._set_state(
                    self.LATCH,
                    f'— docked: range={rng:.3f} m, yaw_err={yaw_err:.3f} rad')
            return
        self.stable_since = None

        # emergency stop on overshoot
        if rng < self.target_range - self.overshoot:
            self.get_logger().warn(
                f'Overshot dock (range {rng:.3f} < {self.target_range:.3f}) — stopping.')
            self._set_state(self.LATCH, '— overshoot stop')
            return

        wz = self.k_yaw * yaw_err
        vx = -self.k_v * (rng - self.target_range) if abs(yaw_err) < self.yaw_gate else 0.0
        self._send(vx, wz)

    # ── LATCH: freeze bot, fire trigger once, wait for the arm ────────────────
    def _latch(self):
        self.state = self.WAIT        # set first: done may arrive before next spin
        trig = Bool(); trig.data = True
        self.trig_pub.publish(trig)
        self._last_trig = self.get_clock().now()
        self.get_logger().info(
            f'Docked — trigger published on /{self.arm_ns}/trigger, '
            f'waiting on /{self.arm_ns}/done')


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
