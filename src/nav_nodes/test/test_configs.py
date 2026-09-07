#!/usr/bin/env python3
"""
Configuration validation tests — catch sim-time config failures in seconds.

A malformed nav2_params.yaml, a map YAML pointing at a missing PGM, or a
xacro that no longer expands all only fail when someone launches the full
stack — i.e. tens of seconds into every sim run, after Gazebo has already
started. These tests validate the static configuration directly so the
failure happens at test time instead.
"""

import os
import shutil
import subprocess

import pytest
import yaml

REPO_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
RD_DIR = os.path.join(REPO_ROOT, 'src', 'robot_description')
NAV2_PARAMS = os.path.join(RD_DIR, 'config', 'nav2_params.yaml')
MAP_YAML = os.path.join(RD_DIR, 'maps', 'large_warehouse.yaml')
ROBOT_XACRO = os.path.join(RD_DIR, 'urdf', 'robot.urdf.xacro')
ARM_XACRO = os.path.join(RD_DIR, 'urdf', 'arm.urdf.xacro')
CONTROLLERS_YAML = os.path.join(RD_DIR, 'config', 'arm_controllers.yaml')

# Core Nav2 nodes the mission node depends on (NavigateToPose action server,
# AMCL pose, costmaps, map server). If one is missing, docking_mission
# stalls forever in INIT.
REQUIRED_NAV2_SECTIONS = {
    'amcl', 'bt_navigator', 'controller_server', 'planner_server',
    'behavior_server', 'local_costmap', 'global_costmap',
    'waypoint_follower', 'velocity_smoother',
}
# NOTE: map_server is intentionally absent — nav2_params.yaml leaves it
# commented out and nav2_bringup supplies it via its launch defaults.


class TestNav2Params:
    def test_parses_as_yaml(self):
        with open(NAV2_PARAMS, encoding='utf-8') as fh:
            params = yaml.safe_load(fh)
        assert isinstance(params, dict) and params

    def test_required_sections_present(self):
        with open(NAV2_PARAMS, encoding='utf-8') as fh:
            params = yaml.safe_load(fh)
        missing = REQUIRED_NAV2_SECTIONS - set(params)
        assert not missing, f'nav2_params.yaml is missing sections: {missing}'

    def test_amcl_initial_pose_matches_spawn(self):
        """
        AMCL set_initial_pose must match the AMR spawn pose.

        warehouse.launch.py spawns at (-4.5, -4.5, yaw 0.9); any mismatch
        means localization is wrong from the first second and the mission
        navigates from a bad belief.
        """
        with open(NAV2_PARAMS, encoding='utf-8') as fh:
            params = yaml.safe_load(fh)
        amcl = params['amcl']['ros__parameters']
        assert amcl['set_initial_pose'] is True
        init = amcl['initial_pose']
        assert init['x'] == pytest.approx(-4.5, abs=1e-3)
        assert init['y'] == pytest.approx(-4.5, abs=1e-3)
        assert init['yaw'] == pytest.approx(0.9, abs=1e-3)


class TestMapYaml:
    def test_map_yaml_resolves_image(self):
        with open(MAP_YAML, encoding='utf-8') as fh:
            map_yaml = yaml.safe_load(fh)
        image = map_yaml['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(MAP_YAML), image)
        assert os.path.isfile(image), (
            f'map image {image!r} missing — map_server fails at launch')

    def test_pgm_dimensions_consistent(self):
        """
        The PGM header must agree with the actual binary payload size.

        A truncated/corrupted PGM makes nav2 crash at load time.
        """
        with open(MAP_YAML, encoding='utf-8') as fh:
            map_yaml = yaml.safe_load(fh)
        assert map_yaml['resolution'] > 0
        image = map_yaml['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(MAP_YAML), image)
        with open(image, 'rb') as fh:
            raw = fh.read()
        # Binary (P5) PGM header: magic line, dims line, maxval line,
        # then exactly width*height grey values.
        try:
            magic, dims_line, maxval_line, payload = raw.split(b'\n', 3)
        except ValueError:
            pytest.fail('map PGM is missing its P5 header lines')
        assert magic.strip() == b'P5'
        width, height = (int(v) for v in dims_line.split())
        assert width > 0 and height > 0
        assert int(maxval_line) == 255
        assert len(payload) >= width * height, (
            f'PGM payload {len(payload)} B smaller than {width}x{height}')


@pytest.mark.skipif(shutil.which('xacro') is None,
                    reason='xacro executable not on PATH')
class TestXacroProcessing:
    """
    Both robot descriptions must still expand.

    A broken xacro kills robot_state_publisher and every spawn action
    before anything moves.
    """

    def _run_xacro(self, args):
        result = subprocess.run(
            ['xacro'] + args, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, (
            f'xacro failed: {result.stderr[-2000:]}')
        assert '<robot' in result.stdout
        return result.stdout

    def test_robot_urdf_expands(self):
        self._run_xacro([ROBOT_XACRO])

    def test_arm_urdf_expands(self):
        """The arm xacro needs the same arguments warehouse.launch.py passes."""
        self._run_xacro([
            ARM_XACRO,
            f'controllers_file:={CONTROLLERS_YAML}',
            'arm_namespace:=arm1',
        ])
