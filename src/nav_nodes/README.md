# `nav_nodes` Package

The `nav_nodes` package contains the high-level autonomy logic for the mobile robot and station arms, including the TF odometry broadcaster, the random-station mission orchestrator, the analytical inverse kinematics pick-and-place service node, and the kinematics test suite.

---

## Package Structure

```
nav_nodes/
├── package.xml               # Dependencies: rclpy, nav2_msgs, trajectory_msgs, std_srvs
├── setup.py                  # Python setuptools entry point definitions
├── setup.cfg                 # Install scripts configuration
├── resource/nav_nodes        # Package marker
├── nav_nodes/                # Autonomy and coordination nodes
│   ├── __init__.py
│   ├── broadcaster_node.py   # odom -> base_footprint TF broadcaster
│   ├── mission_node.py       # Random station/dest delivery state machine
│   └── station_arm_node.py   # Pick-and-place service with analytic IK
└── test/                     # Unit test suite
    ├── test_station_ik.py    # 7 unit tests for kinematics & geometry
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

---

## Executables Registered in `setup.py`

- `broadcaster_node`: Executable for `nav_nodes.broadcaster_node:main`.
- `station_arm_node`: Executable for `nav_nodes.station_arm_node:main`.
- `docking_mission`: Executable for `nav_nodes.mission_node:main`.

---

## Subdirectories

- [`nav_nodes/`](nav_nodes/README.md): Detailed explanation of python node implementations.
- [`test/`](test/README.md): Unit tests and verification scripts.\n