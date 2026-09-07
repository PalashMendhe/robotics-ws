#!/usr/bin/env python3
"""
Pure-logic tests for mission_node.py — no simulator, no DDS peers.

Covers the small pure helpers and module constants of the mission
orchestrator. Why these matter:

  1. _yaw_to_quat() builds the orientation of EVERY NavigateToPose goal.
     A sign flip or non-unit quaternion silently makes Nav2 spin or take
     the long way around — much cheaper to catch here than live.
  2. parse_dock_override() gates hand-tuned dock poses; a regression that
     accepts malformed input would send the AMR to a garbage pose.
  3. DESTINATIONS must cover exactly the two markers defined in
     large_warehouse.sdf (cross-checked in test_geometry_matches_world.py;
     here we assert the invariant the state machine relies on).
"""

import math

from nav_nodes.mission_node import (
    _PHASE_NAMES,
    _yaw_to_quat,
    ABORT,
    CUE,
    DESTINATIONS,
    DONE,
    INIT,
    parse_dock_override,
    TO_DEST,
    TO_STATION,
    WAIT_ARM,
)
import pytest


class TestYawToQuat:
    """Verify the yaw -> quaternion helper used by every Nav2 goal."""

    @pytest.mark.parametrize('yaw,qz,qw', [
        (0.0, 0.0, 1.0),
        (math.pi / 2, math.sqrt(2) / 2, math.sqrt(2) / 2),
        (-math.pi / 2, -math.sqrt(2) / 2, math.sqrt(2) / 2),
    ])
    def test_yaw_to_quat_matches_expected(self, yaw, qz, qw):
        qx, qy, got_qz, got_qw = _yaw_to_quat(yaw)
        assert (qx, qy) == (0.0, 0.0)
        assert got_qz == pytest.approx(qz, abs=1e-9)
        assert got_qw == pytest.approx(qw, abs=1e-9)

    def test_yaw_pi_is_180_degree_rotation(self):
        _, _, qz, qw = _yaw_to_quat(math.pi)
        assert qz == pytest.approx(1.0, abs=1e-9)
        assert qw == pytest.approx(0.0, abs=1e-9)

    @pytest.mark.parametrize('yaw', [-2.7, -0.5, 0.0, 0.31, 1.2, 3.1])
    def test_yaw_to_quat_is_unit_norm(self, yaw):
        """Any non-unit quaternion would be rejected/distorted by Nav2."""
        q = _yaw_to_quat(yaw)
        norm = math.sqrt(sum(c * c for c in q))
        assert norm == pytest.approx(1.0, abs=1e-12)

    def test_quat_rotation_actually_equals_yaw(self):
        """Recover the yaw from the quaternion via atan2 — round-trip check."""
        for yaw in (-2.0, -0.1, 0.0, 0.4, 2.9):
            _, _, qz, qw = _yaw_to_quat(yaw)
            assert 2.0 * math.atan2(qz, qw) == pytest.approx(yaw, abs=1e-12)


class TestParseDockOverride:
    """Verify 'x,y,yaw' parameter parsing for per-station dock overrides."""

    def test_valid_override(self):
        assert parse_dock_override('-4.3,0.228,3.14159') == \
            (-4.3, 0.228, 3.14159)

    def test_valid_override_with_spaces(self):
        assert parse_dock_override(' -4.3 , 0.228 , 3.14 ') == \
            (-4.3, 0.228, 3.14)

    def test_empty_string_means_no_override(self):
        assert parse_dock_override('') is None

    def test_whitespace_only_means_no_override(self):
        assert parse_dock_override('   ') is None

    def test_too_few_fields_rejected(self):
        assert parse_dock_override('-4.3,0.228') is None

    def test_non_numeric_rejected(self):
        assert parse_dock_override('a,b,c') is None

    def test_none_like_input_rejected(self):
        assert parse_dock_override('None') is None


class TestMissionConstants:
    """Invariants the state machine and cross-tests rely on."""

    def test_destinations_cover_both_markers(self):
        assert set(DESTINATIONS.keys()) == {1, 2}

    def test_destinations_inside_map_bounds(self):
        """Map origin (-5.6, -5.6), size 224 px * 0.05 m -> +5.6 m."""
        for x, y, _yaw in DESTINATIONS.values():
            assert -5.6 < x < 5.6
            assert -5.6 < y < 5.6

    def test_phase_names_match_phase_constants(self):
        phases = [INIT, TO_STATION, CUE, WAIT_ARM, TO_DEST, DONE, ABORT]
        assert len(_PHASE_NAMES) == len(phases)
        for phase in phases:
            assert _PHASE_NAMES[phase] == _PHASE_NAMES[phase]  # indexable
        assert _PHASE_NAMES[INIT] == 'INIT'
        assert _PHASE_NAMES[DONE] == 'DONE'
        assert _PHASE_NAMES[ABORT] == 'ABORT'

    def test_phases_are_distinct(self):
        phases = [INIT, TO_STATION, CUE, WAIT_ARM, TO_DEST, DONE, ABORT]
        assert len(set(phases)) == len(phases)
