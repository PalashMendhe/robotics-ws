#!/usr/bin/env python3
"""
Geometry-vs-world consistency tests — the drift guard.

nav_nodes hardcodes the station/parcel/dock/destination geometry that
large_warehouse.sdf and warehouse.launch.py place in the simulation. If
someone moves a box, arm, or marker in the world (or retunes a spawn
coordinate) without updating the Python constants, the mission fails
mysteriously mid-sim: the arm grasps empty air, or the AMR docks at a pose
that no longer exists.

These tests parse the SDF world file and the launch file and assert the
Python constants still agree with the simulation — within 1 mm — so world
edits and code edits can never silently diverge.
"""

import os
import re
import xml.etree.ElementTree as ET

from nav_nodes.mission_node import DESTINATIONS
from nav_nodes.station_arm_node import ARM_X, ARM_Y0, PARCEL_X, PARCEL_Y0
import pytest

REPO_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
SDF_PATH = os.path.join(
    REPO_ROOT, 'src', 'robot_description', 'worlds', 'large_warehouse.sdf')
LAUNCH_PATH = os.path.join(
    REPO_ROOT, 'src', 'robot_description', 'launch', 'warehouse.launch.py')

TOL = 1e-3  # 1 mm — world edits are intentional, not rounding noise


def _sdf_model_pose(root, model_name):
    """Return (x, y, z) of a top-level <model> in the world SDF."""
    for model in root.iter('model'):
        if model.get('name') == model_name:
            pose = model.find('pose')
            assert pose is not None, f'model {model_name} has no <pose>'
            x, y, z = (float(v) for v in pose.text.split()[:3])
            return (x, y, z)
    raise AssertionError(f'model {model_name} not found in {SDF_PATH}')


def _load_sdf():
    tree = ET.parse(SDF_PATH)
    return tree.getroot()


def _station_arm_spawns_from_launch():
    """Pull (x, y, z, yaw) of every make_station_arm() call in the launch file."""
    with open(LAUNCH_PATH, encoding='utf-8') as fh:
        text = fh.read()
    pattern = re.compile(
        r"make_station_arm\([^)]*?x='([^']+)'\s*,\s*y='([^']+)'\s*,"
        r"\s*z='([^']+)'\s*,\s*yaw='([^']+)'", re.DOTALL)
    spawns = [tuple(float(v) for v in m) for m in pattern.findall(text)]
    assert len(spawns) == 3, (
        f'expected 3 station arm spawns in {LAUNCH_PATH}, found {len(spawns)}')
    return spawns


class TestParcelConstantsMatchWorld:
    """station_arm_node PARCEL_* constants vs box_obstacle_<N> SDF poses."""

    def test_parcel_xy_matches_sdf(self):
        root = _load_sdf()
        for station in (1, 2, 3):
            x, y, _z = _sdf_model_pose(root, f'box_obstacle_{station}')
            assert pytest.approx(x, abs=TOL) == PARCEL_X, f'station {station} x'
            expected_y = PARCEL_Y0 + (station - 1) * 1.0
            assert expected_y == pytest.approx(y, abs=TOL), f'station {station} y'

    def test_parcel_spacing_is_one_metre(self):
        """The Python constants encode 1.0 m station spacing (ARM_Y0 += (N-1))."""
        root = _load_sdf()
        ys = [_sdf_model_pose(root, f'box_obstacle_{n}')[1] for n in (1, 2, 3)]
        assert ys[1] - ys[0] == pytest.approx(1.0, abs=TOL)
        assert ys[2] - ys[1] == pytest.approx(1.0, abs=TOL)

    def test_parcel_z_is_grounded_cube_centre(self):
        """
        Parcel z must stay 0.055 (0.1 m cube on the floor).

        The grasp height PICK_GRASP_Z and gripper-clearance tests depend
        on it.
        """
        root = _load_sdf()
        for station in (1, 2, 3):
            _x, _y, z = _sdf_model_pose(root, f'box_obstacle_{station}')
            assert z == pytest.approx(0.055, abs=TOL)


class TestDestinationMarkersMatchWorld:
    """mission_node DESTINATIONS vs destination_marker_<N> SDF poses."""

    def test_destinations_match_sdf_markers(self):
        root = _load_sdf()
        for n in (1, 2):
            x, y, _z = _sdf_model_pose(root, f'destination_marker_{n}')
            dx, dy, _yaw = DESTINATIONS[n]
            assert dx == pytest.approx(x, abs=TOL), f'destination {n} x'
            assert dy == pytest.approx(y, abs=TOL), f'destination {n} y'


class TestArmSpawnsMatchLaunch:
    """station_arm_node ARM_* constants vs warehouse.launch.py arm spawns."""

    def test_arm_xy_matches_launch(self):
        spawns = _station_arm_spawns_from_launch()
        for station, (x, y, _z, _yaw) in enumerate(spawns, start=1):
            assert pytest.approx(x, abs=TOL) == ARM_X, f'station {station} x'
            expected_y = ARM_Y0 + (station - 1) * 1.0
            assert expected_y == pytest.approx(y, abs=TOL), f'station {station} y'

    def test_arm_spawn_yaw_is_pi(self):
        """Arms face -X (yaw = pi); the IK geometry assumes this frame."""
        for _x, _y, _z, yaw in _station_arm_spawns_from_launch():
            assert yaw == pytest.approx(math_pi, abs=1e-3)


math_pi = 3.14159  # launch file uses the literal 3.14159


class TestArmSpawnHeightMatchesIk:
    """
    The IK waypoints are TCP heights above the arm spawn z.

    Launch must keep spawning arms at z = 0.03 or every waypoint shifts
    in world.
    """

    def test_arm_spawn_z_is_0p03(self):
        for _x, _y, z, _yaw in _station_arm_spawns_from_launch():
            assert z == pytest.approx(0.03, abs=TOL)
