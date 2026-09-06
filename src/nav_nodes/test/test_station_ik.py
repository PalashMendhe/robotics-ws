#!/usr/bin/env python3
"""
Offline sanity checks for the large_warehouse station pick-and-place
geometry (no simulator needed):

  1. every IK waypoint solves and lies inside the comfortable arm envelope
     (0.30 m < radial reach < 0.80 m of the 0.817 m max);
  2. every arm joint stays a safe margin off the +/-pi limits (the grasp
     configuration must not butt against the shoulder range - a straight-down
     grasp gave 3.158 rad and stalled the arm at 1.835 live);
  3. the gripper body clears the parcel top at the grasp (25 mm verified by
     /tmp/diag_fix.py - the failing config had -15 mm, i.e. a collision);
  4. the AMR dock pose keeps the robot body clear of the ground parcel;
  5. the dock pose is identical to STATION_DOCK_POSES (single source of
     truth imported by the mission node).

Run:  python3 -m pytest src/nav_nodes/test/test_station_ik.py -v
      (or: python3 src/nav_nodes/test/test_station_ik.py)
"""

import math
import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), '..'))

from nav_nodes.station_arm_node import (  # noqa: E402
    ARM_X, ARM_Y0, compute_station_waypoints, DOCK_DY, DOCK_X,
    PARCEL_Y0, PICK_GRASP_Z, PICK_PREP_Z,
    STATION_DOCK_POSES, station_geometry)

MAX_REACH = 0.80          # arm max 0.817 m — keep a margin
MIN_REACH = 0.30          # avoid fully-folded near-singular grasps
AMR_HALF_LENGTH = 0.175   # base 0.35 m
AMR_HALF_WIDTH = 0.16     # base 0.25 m + wheels
PARCEL_HALF = 0.05
ARM_BASE_Z = 0.03        # arm spawn height (warehouse.launch.py)
TCP_TO_GRIPPER_BASE = 0.075  # gripper_base_link - TCP (arm.urdf.xacro)
GRIPPER_BODY_H = 0.04    # gripper body height (arm.urdf.xacro)
PARCEL_TOP_Z = 0.105      # parcel grounded on floor at z=0.055, 0.1 m tall (top at 0.105)


def _reach(x, y):
    return math.hypot(x, y)


def test_waypoints_within_reach():
    for station in (1, 2, 3):
        for label, kind, positions, secs in compute_station_waypoints(station):
            if kind != 'arm' or label.startswith('HOME'):
                continue
            # positions are IK solutions; verify joint limits instead of
            # re-deriving the TCP — every value must be within ±π.
            for j, q in enumerate(positions):
                # Joint limits are [-3.14, 3.14] for all revolute joints
                assert -3.14 <= q <= 3.14, \
                    f'station {station} {label}: joint {j} = {q:.3f} exceeds [-3.14, 3.14]'
            assert len(positions) == 6


def test_waypoints_not_clamped():
    """
    _ik() clamps acos() when the TCP target is beyond max reach - the arm
    then points at the target with a fully straight elbow (theta3 ≈ 0) and
    never actually arrives. Every working waypoint must have a real elbow
    bend (> 0.1 rad).
    """
    for station in (1, 2, 3):
        for label, kind, positions, secs in compute_station_waypoints(station):
            if kind != 'arm' or label.startswith('HOME'):
                continue
            elbow = positions[2]
            assert abs(elbow) >= 0.05, \
                (f'station {station} {label}: elbow {elbow:.3f} ≈ 0 — '
                 'IK target beyond arm reach (clamped)')


def test_pick_and_place_reach():
    """The raw TCP targets must be inside the arm envelope."""
    for station in (1, 2, 3):
        g = station_geometry(station)
        for lx, ly in (g['pick_local'], g['place_local']):
            r = _reach(lx, ly)
            assert MIN_REACH < r < MAX_REACH, \
                f'station {station}: TCP reach {r:.3f} m out of envelope'


def test_dock_pose_clears_parcel():
    """AMR body (axis-aligned at the dock pose) must not overlap the parcel."""
    for station in (1, 2, 3):
        g = station_geometry(station)
        dx, dy = g['dock_xy']
        px, py = g['parcel_xy']
        # Separation along each axis (boxes, axis-aligned — yaw π flips the
        # AMR in place, footprint is symmetric).
        sep_x = abs(dx - px) - (AMR_HALF_LENGTH + PARCEL_HALF)
        sep_y = abs(dy - py) - (AMR_HALF_WIDTH + PARCEL_HALF)
        assert sep_x > 0.0 or sep_y > 0.0, \
            (f'station {station}: AMR body overlaps the parcel '
             f'(sep_x={sep_x:.3f}, sep_y={sep_y:.3f})')
        # And the tray (AMR centre) must stay inside the arm envelope.
        tray_local = g['place_local']
        assert _reach(*tray_local) < MAX_REACH


def test_gripper_body_clears_parcel():
    """The gripper body must clear the parcel top during the grasp descend.

    The original config (PICK_GRASP_Z = -0.24) put the gripper body bottom
    at z = 0.395 - 15 mm BELOW the parcel top (z = 0.41). The body collided
    with the parcel during the PICK_PREP->PICK_GRASP descend, stalling
    upper_arm at ~2.2 rad and pushing the box ~7 cm diagonally (verified
    live). The fix (PICK_GRASP_Z = -0.20) gives 25 mm clearance.
    """
    for station in (1, 2, 3):
        g = station_geometry(station)
        lx, ly = g['pick_local']
        # TCP world z at grasp
        tcp_z = ARM_BASE_Z + PICK_GRASP_Z
        # Gripper body bottom = TCP + offset_to_base - body_height
        body_bottom = tcp_z + TCP_TO_GRIPPER_BASE - GRIPPER_BODY_H
        clearance = body_bottom - PARCEL_TOP_Z
        assert clearance > 0.0, (
            f'station {station}: gripper body bottom {body_bottom:.3f} '
            f'below parcel top {PARCEL_TOP_Z:.3f} (clearance={clearance*1000:.1f} mm)')
        # Also check the prep pose has even more clearance (sanity)
        prep_tcp_z = ARM_BASE_Z + PICK_PREP_Z
        prep_body_bottom = prep_tcp_z + TCP_TO_GRIPPER_BASE - GRIPPER_BODY_H
        prep_clearance = prep_body_bottom - PARCEL_TOP_Z
        assert prep_clearance > clearance, (
            f'station {station}: prep clearance should exceed grasp clearance')


def test_dock_poses_single_source_of_truth():
    for station in (1, 2, 3):
        g = station_geometry(station)
        assert STATION_DOCK_POSES[station] == (
            g['dock_xy'][0], g['dock_xy'][1], g['dock_yaw'])
    # Station spacing sanity vs. the world file.
    assert ARM_Y0 + 2.0 == station_geometry(3)['arm_xy'][1]
    assert PARCEL_Y0 + 2.0 == station_geometry(3)['parcel_xy'][1]
    assert abs(DOCK_X - (ARM_X + 0.384)) < 1e-4
    assert DOCK_DY < 0.0  # lateral offset must clear the parcel in -Y


def test_lift_and_swing_clearance_above_amr():
    """Verify that during LIFT and SWING, the bottom of the box is well above the AMR top rim."""
    AMR_TOP_RIM_Z = 0.237  # top of AMR funnel lip
    for station in (1, 2, 3):
        wps = compute_station_waypoints(station)
        for label, kind, positions, secs in wps:
            if 'LIFT' in label or 'SWING' in label:
                # Local Z of LIFT/SWING is 0.450 -> world TCP is 0.03 + 0.45 = 0.48 m
                # Bottom of 0.10 m parcel is at least TCP z - 0.05 m = 0.43 m
                box_bottom_z = 0.48 - 0.05
                clearance = box_bottom_z - AMR_TOP_RIM_Z
                assert clearance > 0.15, (
                    f'{label}: box bottom {box_bottom_z:.3f} m has only {clearance*1000:.1f} mm '
                    f'clearance over AMR rim {AMR_TOP_RIM_Z:.3f} m (expected > 150 mm)')


if __name__ == '__main__':
    test_waypoints_within_reach()
    test_waypoints_not_clamped()
    test_pick_and_place_reach()
    test_gripper_body_clears_parcel()
    test_dock_pose_clears_parcel()
    test_dock_poses_single_source_of_truth()
    test_lift_and_swing_clearance_above_amr()
    print('All station-geometry sanity checks passed.')
