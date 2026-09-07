#!/usr/bin/env python3
"""
docking_mission: random-station docking + Nav2 delivery orchestrator.

World: large_warehouse. Run with:
    ros2 run nav_nodes docking_mission
    ros2 run nav_nodes docking_mission --ros-args -p station:=2 -p dest:=1

Flow (one-shot timer phase machine):
    INIT        wait for Nav2 (NavigateToPose action server) + AMCL pose
    SELECT      random station ∈ {1,2,3} and destination ∈ {1,2} (params
                station:/dest: force specific values for debugging)
    TO_STATION  NavigateToPose → station dock pose (map frame). Nav2's goal
                checker (±0.08 m) is the arrival test — no ArUco involved.
    CUE         call /station_<N>/dock_arm (std_srvs/Trigger). The arm ACKs
                instantly and runs its IK pick-and-place; completion arrives
                on /arm<N>/done (latched Bool, armed by an earlier False).
    TO_DEST     NavigateToPose → destination marker centre
    DONE        parcel delivered

Localization: AMCL is initialized from nav2_params.yaml
(set_initial_pose: true at the AMR spawn pose -4.5, -4.5, yaw 0.9); this
node never hardcodes where the bot is — it only issues world-frame goals
and lets Nav2 + AMCL do the rest. Retry policy: one NavigateToPose retry
per leg, one dock-cue retry, then abort with /mission/status.

Dependencies: rclpy, std_msgs, std_srvs, nav2_msgs, geometry_msgs, action_msgs
"""

import math
import os
import random

os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ.setdefault('GZ_IP', '127.0.0.1')

from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav_nodes.station_arm_node import STATION_DOCK_POSES  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

# Destination markers in large_warehouse.sdf (world/map frame).
DESTINATIONS = {1: (4.4586, 4.3795, 0.0), 2: (4.4590, -0.9218, 0.0)}

# Phases
INIT, TO_STATION, CUE, WAIT_ARM, TO_DEST, DONE, ABORT = range(7)
_PHASE_NAMES = ['INIT', 'TO_STATION', 'CUE', 'WAIT_ARM', 'TO_DEST',
                'DONE', 'ABORT']

NAV_POLL_PERIOD = 0.2      # s — phase/future polling rate
CUE_TIMEOUT = 90.0         # s — arm sequence (~25 s) + generous margin


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def parse_dock_override(override):
    """
    Parse a 'x,y,yaw' dock-override string into (x, y, yaw) floats.

    Returns None when the string is empty or malformed. Pure function so
    the parsing policy is unit-testable without a live node.
    """
    override = str(override).strip()
    if not override:
        return None
    try:
        x, y, yaw = (float(v) for v in override.split(','))
        return (x, y, yaw)
    except ValueError:
        return None


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ── parameters ────────────────────────────────────────────────────────
        try:
            if not self.get_parameter('use_sim_time').value:
                self.set_parameters([
                    rclpy.Parameter(
                        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        except Exception:
            pass
        self.declare_parameter('station', 0)   # 0 = random
        self.declare_parameter('dest', 0)      # 0 = random
        # Per-station dock overrides "x,y,yaw" (debugging / hand tuning)
        self.declare_parameter('station_1_dock', '')
        self.declare_parameter('station_2_dock', '')
        self.declare_parameter('station_3_dock', '')

        station = int(self.get_parameter('station').value)
        dest = int(self.get_parameter('dest').value)
        self.station = station if station in (1, 2, 3) else random.choice([1, 2, 3])
        self.dest = dest if dest in (1, 2) else random.choice([1, 2])

        self.dock_poses = dict(STATION_DOCK_POSES)
        for n in (1, 2, 3):
            override = str(self.get_parameter(f'station_{n}_dock').value)
            parsed = parse_dock_override(override)
            if parsed is not None:
                self.dock_poses[n] = parsed
                self.get_logger().warn(
                    f'station_{n}_dock overridden to {parsed}')
            elif override.strip():
                self.get_logger().error(
                    f'Bad station_{n}_dock "{override}" — ignored '
                    '(expected "x,y,yaw")')

        # ── interfaces ────────────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cue_client = self.create_client(
            Trigger, f'/station_{self.station}/dock_arm')
        # Latched done from the station arm (armed by the arm's False latch).
        done_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._done_sub = self.create_subscription(
            Bool, f'/arm{self.station}/done', self._done_cb, done_qos)
        self._status_pub = self.create_publisher(
            String, '/mission/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # ── state ─────────────────────────────────────────────────────────────
        self.phase = INIT
        self._goal_future = None
        self._result_future = None
        self._cue_future = None
        self._cue_pending = False
        self._cue_sent_at = None
        self._nav_retries = 0
        self._cue_retries = 0

        self._publish_status(
            f'STATION_SELECTED station={self.station} dest={self.dest}')
        self.get_logger().info(
            f'Mission: station {self.station} (dock '
            f'{self.dock_poses[self.station]}) → destination {self.dest} '
            f'{DESTINATIONS[self.dest]}')
        self._timer = self.create_timer(NAV_POLL_PERIOD, self._poll)

    # ── helpers ───────────────────────────────────────────────────────────────
    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'[{_PHASE_NAMES[self.phase]}] {text}')

    def _set_phase(self, phase, note=''):
        self.phase = phase
        self.get_logger().info(f'→ phase {_PHASE_NAMES[phase]} {note}')

    def _make_pose(self, xyz_yaw):
        x, y, yaw = xyz_yaw
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        _, _, qz, qw = _yaw_to_quat(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _send_nav_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'NavigateToPose → ({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f})')
        self._goal_future = self._nav_client.send_goal_async(goal)

    def _amcl_alive(self):
        """AMCL publishes /amcl_pose once localized."""
        try:
            return (self.count_publishers('amcl_pose') > 0 or
                    self.count_publishers('/amcl_pose') > 0)
        except Exception:
            return False

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _done_cb(self, msg):
        # Only a True received AFTER the cue was fired (the arm latches False
        # on acceptance, killing any stale True from a previous run) counts.
        if (self.phase == WAIT_ARM and msg.data and self._cue_pending):
            self._cue_pending = False
            self._publish_status('ARM_DONE — parcel loaded, delivering')
            self._set_phase(TO_DEST)
            self._nav_retries = 0
            self._send_nav_goal(self._make_pose(DESTINATIONS[self.dest]))

    # ── main poll loop (drives every phase) ───────────────────────────────────
    def _poll(self):
        if self.phase == INIT:
            if not self._nav_client.wait_for_server(timeout_sec=0.0):
                return
            if not self._amcl_alive():
                return
            self._publish_status('NAV2_READY — navigating to station')
            self._set_phase(TO_STATION)
            self._send_nav_goal(self._make_pose(self.dock_poses[self.station]))

        elif self.phase == TO_STATION:
            self._poll_nav(on_success=self._after_station_arrival,
                           leg='station')

        elif self.phase == CUE:
            if self._cue_future is None:
                if not self._cue_client.service_is_ready():
                    return  # wait for the station node to come up
                self._cue_sent_at = self.get_clock().now()
                self._cue_pending = True
                self._cue_future = self._cue_client.call_async(Trigger.Request())
                self._publish_status(
                    f'CUE_SENT /station_{self.station}/dock_arm')
                return
            if not self._cue_future.done():
                # Timeout guard: the ACK should be near-instant.
                held = (self.get_clock().now() - self._cue_sent_at).nanoseconds / 1e9
                if held > 10.0:
                    self._abort('dock service ACK timeout')
                return
            resp = self._cue_future.result()
            self._cue_future = None
            if resp.success:
                self._set_phase(WAIT_ARM, '— arm picking parcel')
            elif self._cue_retries < 1:
                self._cue_retries += 1
                self.get_logger().warn(
                    f'Dock cue rejected: {resp.message} — retrying once')
            else:
                self._abort(f'dock cue rejected: {resp.message}')

        elif self.phase == WAIT_ARM:
            held = (self.get_clock().now() - self._cue_sent_at).nanoseconds / 1e9
            if held > CUE_TIMEOUT:
                self._abort('arm sequence timed out (no /done)')

        elif self.phase == TO_DEST:
            self._poll_nav(on_success=self._after_delivery, leg='destination')

    # ── NavigateToPose future plumbing ────────────────────────────────────────
    def _poll_nav(self, on_success, leg):
        if self._result_future is None:
            if not self._goal_future.done():
                return
            goal_handle = self._goal_future.result()
            self._goal_future = None
            if not goal_handle.accepted:
                self._nav_failure(leg, 'goal rejected')
                return
            self._result_future = goal_handle.get_result_async()
            return

        if not self._result_future.done():
            return
        result = self._result_future.result()
        self._result_future = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            on_success()
        else:
            self._nav_failure(leg, f'status {result.status}')

    def _nav_failure(self, leg, reason):
        if self._nav_retries < 3:
            self._nav_retries += 1
            self.get_logger().warn(
                f'{leg} navigation failed ({reason}) — retrying ({self._nav_retries}/3)')
            target = (self.dock_poses[self.station] if leg == 'station'
                      else DESTINATIONS[self.dest])
            self._send_nav_goal(self._make_pose(target))
        else:
            self._abort(f'{leg} navigation failed: {reason}')

    # ── phase transitions ─────────────────────────────────────────────────────
    def _after_station_arrival(self):
        self._publish_status('DOCKED — cueing station arm')
        self._set_phase(CUE)

    def _after_delivery(self):
        self._publish_status(
            f'DELIVERED — station {self.station} → destination {self.dest}')
        self._set_phase(DONE)

    def _abort(self, reason):
        self._publish_status(f'ABORTED — {reason}')
        self._set_phase(ABORT)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
