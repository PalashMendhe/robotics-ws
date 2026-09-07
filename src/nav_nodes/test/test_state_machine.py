#!/usr/bin/env python3
"""
State-machine transition tests for MissionNode — no simulator, no Nav2.

The mission orchestrator's retry/timeout policy is exactly the logic that
cannot be exercised by hand: each sim run takes minutes and the failure
paths (rejected dock cue, nav goal rejection, arm that never finishes) are
rare and awkward to reproduce. These tests drive the phase machine with
stubbed action/service clients so every transition and retry decision is
verified deterministically:

  * INIT only leaves once Nav2 AND AMCL are alive;
  * nav failure retries up to 3 times, then ABORTs (never retries forever);
  * a rejected dock cue retries exactly once, then ABORTs;
  * a dock-service ACK timeout ABORTs (guard against a dead station node);
  * /armN/done=True is only honoured in WAIT_ARM after the cue was sent —
    a stale latched True from a previous run must never advance the mission;
  * WAIT_ARM has a hard CUE_TIMEOUT abort.
"""

from nav_nodes import mission_node as mn
from nav_nodes.mission_node import (
    ABORT,
    CUE,
    CUE_TIMEOUT,
    INIT,
    MissionNode,
    TO_DEST,
    WAIT_ARM,
)
import pytest
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class _StubFuture:
    """Minimal future: preset outcome, done() switchable."""

    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        if not self._done:
            raise RuntimeError('stub future not done')
        return self._result


class _StubNavClient:
    """Records NavigateToPose goals instead of touching DDS."""

    def __init__(self):
        self.goals = []

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _StubFuture(done=False)  # goal never resolves in these tests


class _StubCueClient:
    def service_is_ready(self):
        return True

    def call_async(self, request):
        raise AssertionError('cue should be stubbed via _cue_future')


class _StaticClock:
    """
    Wall clock frozen at t=100 s for deterministic timeout logic.

    The node enables sim time, which reports t=0 until /clock exists.
    """

    def now(self):
        return Time(nanoseconds=100_000_000_000)


@pytest.fixture()
def rclpy_ctx():
    mn.rclpy.init()
    yield
    mn.rclpy.shutdown()


@pytest.fixture()
def mission(rclpy_ctx):
    node = MissionNode()
    node._nav_client = _StubNavClient()
    node._cue_client = _StubCueClient()
    yield node
    node.destroy_node()


def _cue_future(success=True, message='sequence started', done=True):
    return _StubFuture(
        result=Trigger.Response(success=success, message=message), done=done)


class TestInitPhase:
    def test_init_waits_for_nav2_and_amcl(self, mission):
        assert mission.phase == INIT

        # Nav2 not up yet -> stay in INIT
        mission._nav_client.wait_for_server = lambda timeout_sec=0.0: False
        mission._poll()
        assert mission.phase == INIT

        # Nav2 up, but AMCL has not localized -> stay in INIT
        mission._nav_client.wait_for_server = lambda timeout_sec=0.0: True
        mission.count_publishers = lambda topic: 0
        mission._poll()
        assert mission.phase == INIT

        # Both alive -> leave INIT for TO_STATION and send one goal
        mission.count_publishers = lambda topic: 1
        mission._poll()
        assert mission.phase == mn.TO_STATION
        assert len(mission._nav_client.goals) == 1

    def test_station_and_dest_are_valid(self, mission):
        assert mission.station in (1, 2, 3)
        assert mission.dest in (1, 2)
        assert mission.dock_poses[mission.station] in \
            tuple(mn.STATION_DOCK_POSES.values())


class TestNavRetryPolicy:
    def test_retries_up_to_three_then_aborts(self, mission):
        mission.phase = mn.TO_STATION
        mission._nav_retries = 0
        for _ in range(3):
            mission._nav_failure('station', 'status 1')
            assert mission.phase != ABORT
        assert mission._nav_retries == 3
        assert len(mission._nav_client.goals) == 3
        # 4th failure (retry budget exhausted) -> abort
        mission._nav_failure('station', 'status 1')
        assert mission.phase == ABORT
        # no extra goal is sent once aborted
        assert len(mission._nav_client.goals) == 3

    def test_nav_retry_counter_resets_after_arm_done(self, mission):
        mission._nav_retries = 2
        mission.phase = WAIT_ARM
        mission._cue_pending = True
        mission._done_cb(Bool(data=True))
        assert mission.phase == TO_DEST
        assert mission._nav_retries == 0


class TestCuePolicy:
    def test_rejected_cue_retries_once_then_aborts(self, mission):
        mission.phase = CUE
        mission._cue_sent_at = mission.get_clock().now()
        mission._cue_retries = 0

        # First rejection -> one retry, stay in CUE
        mission._cue_future = _cue_future(success=False, message='busy')
        mission._poll()
        assert mission.phase == CUE
        assert mission._cue_retries == 1
        assert mission._cue_future is None  # consumed; next poll re-sends

        # Second rejection -> abort
        mission._cue_future = _cue_future(success=False, message='busy')
        mission._poll()
        assert mission.phase == ABORT

    def test_accepted_cue_advances_to_wait_arm(self, mission):
        mission.phase = CUE
        mission._cue_sent_at = mission.get_clock().now()
        mission._cue_future = _cue_future(success=True)
        mission._poll()
        assert mission.phase == WAIT_ARM

    def test_ack_timeout_aborts(self, mission):
        """
        The dock ACK should be near-instant.

        A 10 s silence means the station node died and the mission must
        not hang in CUE forever.
        """
        mission.phase = CUE
        mission._cue_future = _cue_future(done=False)
        mission.get_clock = lambda: _StaticClock()
        mission._cue_sent_at = _StaticClock().now() - Duration(seconds=11.0)
        mission._poll()
        assert mission.phase == ABORT

    def test_wait_arm_timeout_aborts(self, mission):
        mission.phase = WAIT_ARM
        mission._cue_pending = True
        mission.get_clock = lambda: _StaticClock()
        mission._cue_sent_at = _StaticClock().now() - Duration(
            seconds=CUE_TIMEOUT + 1.0)
        mission._poll()
        assert mission.phase == ABORT

    def test_wait_arm_under_timeout_keeps_waiting(self, mission):
        """
        Just inside CUE_TIMEOUT the mission must NOT abort.

        Guards the off-by-one that would kill legitimate ~25 s sequences.
        """
        mission.phase = WAIT_ARM
        mission._cue_pending = True
        mission.get_clock = lambda: _StaticClock()
        mission._cue_sent_at = _StaticClock().now() - Duration(
            seconds=CUE_TIMEOUT - 5.0)
        mission._poll()
        assert mission.phase == WAIT_ARM


class TestArmDoneHandshake:
    def test_stale_true_is_ignored(self, mission):
        """
        A latched /armN/done=True from a previous run must not advance.

        The True only counts in WAIT_ARM after the cue was actually sent.
        """
        mission.phase = WAIT_ARM
        mission._cue_pending = False
        mission._done_cb(Bool(data=True))
        assert mission.phase == WAIT_ARM

    def test_true_in_wrong_phase_is_ignored(self, mission):
        mission.phase = mn.TO_STATION
        mission._cue_pending = True
        mission._done_cb(Bool(data=True))
        assert mission.phase == mn.TO_STATION

    def test_true_after_cue_drives_to_destination(self, mission):
        mission.phase = WAIT_ARM
        mission._cue_pending = True
        mission._done_cb(Bool(data=True))
        assert mission.phase == TO_DEST
        assert len(mission._nav_client.goals) == 1
        goal = mission._nav_client.goals[0]
        dx, dy, _ = mn.DESTINATIONS[mission.dest]
        assert goal.pose.pose.position.x == pytest.approx(dx, abs=1e-6)
        assert goal.pose.pose.position.y == pytest.approx(dy, abs=1e-6)
        assert goal.pose.header.frame_id == 'map'


class TestPhaseOrdering:
    def test_phase_indices(self):
        """
        Document the intended phase order.

        A refactor that reorders the range() constants must be a
        conscious decision, not an accident.
        """
        assert mn.INIT < mn.TO_STATION < mn.CUE < mn.WAIT_ARM \
            < mn.TO_DEST < mn.DONE < mn.ABORT
