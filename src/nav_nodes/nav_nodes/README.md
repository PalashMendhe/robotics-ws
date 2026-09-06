# `src/nav_nodes/nav_nodes/`

This directory contains the Python node implementations executing the autonomous mission, station arm manipulation, and robot coordinate transformations.

---

## Files

### 1. `mission_node.py` (Registered Executable: `docking_mission`)
- **What it does**:
  - Top-level mission orchestrator state machine.
  - **Execution Flow**:
    1. `INIT`: Connects to Nav2 `NavigateToPose` action server and confirms AMCL pose convergence.
    2. `SELECT`: Randomly picks one of 3 stations (1, 2, or 3) and one of 2 delivery destinations (1 or 2).
    3. `TO_STATION`: Dispatches `NavigateToPose` goal to the selected station dock pose `(DOCK_X, dock_y, yaw=pi)`.
    4. `CUE`: Calls `/station_<N>/dock_arm` (`std_srvs/Trigger`) to signal the station manipulator.
    5. `WAIT_ARM`: Monitors latched topic `/arm<N>/done` until the arm signals completion (`True`).
    6. `TO_DEST`: Dispatches `NavigateToPose` goal to the chosen destination marker coordinates.
    7. `DONE`: Emits `/mission/status` topic status `"DELIVERED"`.
  - **Parameters**: Supports manual overrides via `-p station:=<1|2|3>` and `-p dest:=<1|2>`.
- **Requirement**: Primary high-level mission controller driving the AMR.

---

### 2. `station_arm_node.py` (Registered Executable: `station_arm_node`)
- **What it does**:
  - Per-station pick-and-place service provider (`/station_<N>/dock_arm`).
  - **Geometry & Waypoints**:
    - Maintains single source of truth for dock coordinates (`DOCK_X = -4.300\text{ m}`, `DOCK_DY = -0.3000\text{ m}`).
    - Re-solves exact analytical inverse kinematics keeping all 6 revolute joints strictly in $[-3.14, 3.14]$ radians and keeping the elbow safely elevated ($z > 0.45\text{ m}$).
  - **Streamlined 7-Step Sequence**:
    1. `OPEN`: Opens gripper prongs to $0.12\text{ m}$ span (`[0.06, -0.06]`).
    2. `PICK_GRASP`: Forearm reaches down directly over floor parcel at $z=0.10\text{ m}$.
    3. `GRIP`: Clamps prongs firmly on parcel (`[0.018, -0.018]`).
    4. `LIFT`: Lifts parcel pure vertically to $z=0.48\text{ m}$ ($+193\text{ mm}$ clearance above AMR rim).
    5. `SWING`: Base rotates to $-0.8888\text{ rad}$ at high altitude ($z=0.48\text{ m}$) centering over the AMR tray.
    6. `LOWER`: Forearm descends pure vertically ($q_0 = -0.8888$) into tray funnel at $z=0.28\text{ m}$.
    7. `RELEASE`: Gripper opens (`[0.06, -0.06]`), releasing the parcel cleanly onto the tray floor.
  - Publishes joint trajectories directly to `/<arm_ns>/arm_controller/joint_trajectory` and `/<arm_ns>/gripper_controller/joint_trajectory`.
  - Signals completion on `/<arm_ns>/done` (`std_msgs/Bool`, latched).
- **Requirement**: Autonomous manipulation service for grasping parcels and loading the AMR tray.

---

### 3. `broadcaster_node.py` (Registered Executable: `broadcaster_node`)
- **What it does**:
  - Subscribes to `/model/my_robot/odometry` from Gazebo.
  - Broadcasts dynamic `odom -> base_footprint` TF transform to the ROS 2 `/tf` tree.
  - Enforces strictly monotonic timestamps to eliminate `TF_OLD_DATA` warnings.
- **Requirement**: Mandatory for Nav2 and AMCL coordinate frame transformations.

---

### 4. `__init__.py`
- Package initialization marker for Python imports.\n