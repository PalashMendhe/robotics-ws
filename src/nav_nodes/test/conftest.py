#!/usr/bin/env python3
"""
Shared pytest configuration for the nav_nodes headless test suite.

Puts the `nav_nodes` package on sys.path (mirroring the trick used by
test_station_ik.py) so tests can import the nodes directly from the source
tree without a colcon build. Tests in this directory are designed to run
WITHOUT a running simulator; ROS-dependent tests import rclpy but never
require DDS peers, a Gazebo instance, or /clock.
"""

import os
import sys

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(TEST_DIR, '..')
if PKG_DIR not in sys.path:
    sys.path.insert(0, os.path.abspath(PKG_DIR))

REPO_ROOT = os.path.abspath(os.path.join(PKG_DIR, '..', '..'))

# Keep test DDS traffic isolated from anything the developer has running.
os.environ.setdefault('ROS_DOMAIN_ID', '77')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ.setdefault('RCUTILS_COLORIZED_OUTPUT', '0')
