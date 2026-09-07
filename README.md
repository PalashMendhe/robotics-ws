# Autonomous Mobile Robot (AMR) & Multi-Station Warehouse Logistics System

An end-to-end, autonomous warehouse logistics solution built with **ROS 2 Jazzy** and **Gazebo Harmonic**. The system coordinates an Autonomous Mobile Robot (AMR) with multiple 6-DOF robotic station manipulators to autonomously fulfill random pick-and-deliver missions across a multi-aisle industrial warehouse.

---

## System Overview

The project simulates an industrial warehouse logistics workflow:
1. **Autonomous Mission Orchestration**: A central mission state machine randomly selects an active fulfillment station (1, 2, or 3) and a destination marker (1 or 2).
2. **Nav2 Autonomous Navigation**: The AMR drives from its home pad through structured aisles to the chosen station docking position using **AMCL localization** on a solid rasterized 2D occupancy grid map.
3. **Trigger-Based Station Cueing**: Upon arrival, the AMR triggers the station arm via a ROS 2 service call (`/station_<N>/dock_arm`).
4. **Analytical-IK Pick-and-Place Manipulation**: The grounded 6-DOF station arm grasps a physical delivery box from the floor, lifts it with $+193\text{ mm}$ clearance above the robot, swings across, and lowers it pure vertically into the AMR's mechanical funnel tray.
5. **Pure-Physics Parcel Retention**: The AMR transports the carried parcel across the warehouse with zero artificial coordinate locking, relying strictly on physics-based retention (ODE friction and angled funnel walls).
6. **Final Delivery**: The AMR parks on the destination marker and marks the mission as `DELIVERED`.

```
                  ┌────────────────────────────────────────────────────────┐
                  │                 MISSION STATE MACHINE                  │
                  └────────────────────────────────────────────────────────┘
                                              │
                                              ▼
                                    ┌──────────────────┐
                                    │       INIT       │  Select station & destination
                                    └──────────────────┘  Wait for Nav2 active & AMCL
                                              │
                                              ▼
                                    ┌──────────────────┐
                                    │    TO_STATION    │  NavigateToPose → (-4.300, dock_y, π)
                                    └──────────────────┘
                                              │ Arrived (tolerance ±0.08 m)
                                              ▼
                                    ┌──────────────────┐
                                    │       CUE        │  std_srvs/Trigger → /station_N/dock_arm
                                    └──────────────────┘
                                              │ Service ACK received
                                              ▼
                                    ┌──────────────────┐
                                    │     WAIT_ARM     │  Station arm executes 7-step sequence
                                    └──────────────────┘  Places parcel in AMR tray
                                              │ /armN/done == True
                                              ▼
                                    ┌──────────────────┐
                                    │     TO_DEST      │  NavigateToPose → Destination marker
                                    └──────────────────┘  (Pure physics parcel retention)
                                              │ Arrived at destination
                                              ▼
                                    ┌──────────────────┐
                                    │       DONE       │  Publish status "DELIVERED"
                                    └──────────────────┘
```

---

## Packages in Workspace

| Package | Path | Description |
|:---|:---|:---|
| **`robot_description`** | [`src/robot_description/`](src/robot_description/README.md) | URDF descriptions for AMR (skid-steer ODE friction tuning, sensor suite, funnel tray) and 6-DOF Station Arm (grounded pedestal, `ros2_control`), Gazebo Harmonic warehouse world, Nav2 solid rasterized maps, and master bringup launch files. |
| **`nav_nodes`** | [`src/nav_nodes/`](src/nav_nodes/README.md) | High-level autonomy package: `docking_mission` (state machine orchestrator), `station_arm_node` (analytical IK pick-and-place service), `broadcaster_node` (monotonic TF odometry broadcaster), and Pytest kinematics verification suite. |
| **`arm_moveit_config`** | [`src/arm_moveit_config/`](src/arm_moveit_config/README.md) | MoveIt 2 motion planning configuration for the 6-DOF manipulator: SRDF semantic model, KDL kinematics solver, joint limits, controller bindings, and OMPL algorithms. |
| **`robotic_4dof_arm`** | [`src/robotic_4dof_arm/`](src/robotic_4dof_arm/README.md) | Standalone 6-DOF manipulator simulation package: URDF model, controllers, MoveIt autonomous pick-and-place scripts, collision scene manager, and RViz marker publishers. |

---

## Directory Sitemap & Documentation

Every folder in this repository contains a dedicated `README.md` detailing its component files and technical requirements:

- **`src/robot_description/`** ([README](src/robot_description/README.md))
  - [`urdf/`](src/robot_description/urdf/README.md) — AMR chassis (`robot.urdf.xacro`) and 6-DOF station manipulator (`arm.urdf.xacro`).
  - [`launch/`](src/robot_description/launch/README.md) — Master mission launcher (`mission.launch.py`), simulation bringup (`warehouse.launch.py`), and Nav2 bringup (`nav2.launch.py`).
  - [`config/`](src/robot_description/config/README.md) — Nav2 parameters (`nav2_params.yaml`), controller manager config (`arm_controllers.yaml`), and direct topic bridge (`joint_bridge.yaml`).
  - [`maps/`](src/robot_description/maps/README.md) — Solid rasterized 2D occupancy grid map (`large_warehouse.yaml`, `large_warehouse.pgm`).
  - [`worlds/`](src/robot_description/worlds/README.md) — Gazebo Harmonic world model (`large_warehouse.sdf`) with shelves, physical floor boxes, and destination pads.
- **`src/nav_nodes/`** ([README](src/nav_nodes/README.md))
  - [`nav_nodes/`](src/nav_nodes/nav_nodes/README.md) — Mission state machine (`mission_node.py`), station arm service (`station_arm_node.py`), TF broadcaster (`broadcaster_node.py`).
  - [`test/`](src/nav_nodes/test/README.md) — Pytest test suite (`test_station_ik.py`) verifying joint limits, reach envelopes, and box clearances.
- **`src/arm_moveit_config/`** ([README](src/arm_moveit_config/README.md))
  - [`config/`](src/arm_moveit_config/config/README.md) — MoveIt SRDF (`arm.srdf`), joint limits, KDL kinematics, controller bindings, and OMPL algorithms.
  - [`launch/`](src/arm_moveit_config/launch/README.md) — MoveIt 2 bringup with RViz motion planning (`moveit.launch.py`).
- **`src/robotic_4dof_arm/`** ([README](src/robotic_4dof_arm/README.md))
  - [`config/`](src/robotic_4dof_arm/config/README.md) — Standalone controller configuration and RViz display layout.
  - [`launch/`](src/robotic_4dof_arm/launch/README.md) — Standalone Gazebo simulation and GUI display launchers.
  - [`scripts/`](src/robotic_4dof_arm/scripts/README.md) — MoveIt autonomous pick-and-place (`pick_and_place.py`), collision scene manager, and RViz markers.
  - [`urdf/`](src/robotic_4dof_arm/urdf/README.md) — Standalone 6-DOF manipulator Xacro description.
  - [`worlds/`](src/robotic_4dof_arm/worlds/README.md) — Standalone arm test world.

---

## Tech Stack

- **Operating System**: Linux (Ubuntu 24.04 LTS)
- **ROS 2 Distribution**: ROS 2 Jazzy Jalisco
- **Physics Simulator**: Gazebo Harmonic (ODE Physics Engine, step size $0.001\text{ s}$)
- **Navigation & Localization**: Nav2 (AMCL, Navfn Global Planner, Costmap2D, RotationShimController, Velocity Smoother)
- **Manipulation & Motion Planning**: MoveIt 2 (OMPL, KDL Kinematics) & Custom Analytical Closed-Form IK
- **Hardware Abstraction**: `ros2_control` (`gz_ros2_control`, `joint_trajectory_controller`, `joint_state_broadcaster`)
- **Languages**: Python 3.12, C++17, URDF / Xacro, SDF

---

## Quick Start Guide

### 1. Build the Workspace
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch the Full Warehouse Simulation
```bash
export GZ_IP=127.0.0.1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Spawns large_warehouse, AMR at home pad, 3 station arms with controllers, and Nav2
ros2 launch robot_description mission.launch.py headless:=true
```

### 3. Run Autonomous Pick-and-Delivery Mission
In a new terminal:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run random station and random destination mission:
ros2 run nav_nodes docking_mission

# Or force specific station (1, 2, or 3) and destination (1 or 2):
ros2 run nav_nodes docking_mission --ros-args -p station:=1 -p dest:=1
```

### 4. Run Standalone MoveIt 2 Arm Motion Planning
```bash
# Terminal 1: Launch standalone arm simulation in Gazebo
ros2 launch robotic_4dof_arm gazebo.launch.py

# Terminal 2: Launch MoveIt 2 planning pipeline with RViz
ros2 launch arm_moveit_config moveit.launch.py use_rviz:=true

# Terminal 3: Run autonomous MoveIt pick-and-place script
ros2 run robotic_4dof_arm pick_and_place.py
```

### 5. Run Kinematic Verification Tests
```bash
pytest src/nav_nodes/test/test_station_ik.py
```

---

## Technical Decisions & Problem Resolutions

The system design reflects key technical decisions and solutions derived from rigorous simulation debugging (detailed in [`bug.md`](bug.md)):

1. **Decoupled Skid-Steer Friction (ODE `<fdir1>`)**:
   - Standard cylinder contact in Gazebo Harmonic applies isotropic friction, causing 4-wheel skid-steer bots to stall during in-place turns. Adding `<fdir1>1 0 0</fdir1>` separates rolling friction from lateral sliding friction (`<mu2>0.05</mu2>`), enabling pure in-place pivoting ($\omega_z = 1.2\text{ rad/s}$) without drift.
2. **Solid Rasterized Obstacle Footprints**:
   - Navfn cuts through hollow shelf interiors mapped by standard raycasting. By computing exact 2D bounding boxes and rasterizing all 17 shelves as 100% solid occupied cells (`0`), the global planner routes strictly through open aisles.
3. **Monotonic Single-Source TF Tree**:
   - To eliminate transform race conditions between `robot_localization` EKF and Gazebo odometry, EKF was omitted. A dedicated `broadcaster_node` subscribes to true odometry and broadcasts `odom -> base_footprint` with strictly monotonic timestamps, completely resolving particle filter jitter in AMCL.
4. **Gentle Navigation Dynamics & Mechanical Retention**:
   - Pure-physics box transport requires avoiding aggressive linear/angular jerk. We configured Nav2 `RotationShimController` to align the AMR with its path before advancing, capped acceleration at $2.0\text{ m/s}^2$ with `velocity_smoother`, and built an integrated funnel tray with $15^\circ$ angled lip walls to retain the box mechanically.
5. **Grounded Arm Pedestals & `<ros2_control>` Stability**:
   - Station manipulators are mounted on grounded steel pedestals ($0.30\text{ m} \times 0.30\text{ m} \times 0.03\text{ m}$) flush with the floor at $z = 0.03\text{ m}$. Initial controller interfaces default cleanly to `0.0` (upright safe stance) with joint damping ($5.0\text{ Nms/rad}$) preventing gravity droop.
6. **Analytical Elbow-Up Inverse Kinematics within Strict Limits**:
   - Pick-and-place trajectories enforce an elbow-up stance ($z_{elbow} > 0.45\text{ m}$, $z_{wrist} > 0.45\text{ m}$) to prevent the manipulator from colliding with the floor plane. All 6 revolute joints strictly operate within $[-3.14, 3.14]$ radians.
7. **Two-Stage Vertical Lift & High-Altitude Swing**:
   - Diagonal transfers between floor and AMR tray risk scraping the robot chassis. The arm executes a pure vertical `LIFT` at the parcel coordinates to TCP $z = 0.480\text{ m}$ ($+193\text{ mm}$ clearance above the AMR rim), followed by high-clearance horizontal `SWING`, and pure vertical `LOWER`.
8. **Precision Docking ($x = -4.300\text{ m}$) & Pure Vertical Tray Placement**:
   - AMR docking standoff is calibrated to $x = -4.300\text{ m}$ (front bumper clears the pedestal by $5.9\text{ cm}$). Radial reach to the tray ($r = 0.487\text{ m}$) matches the pick radius ($r = 0.492\text{ m}$). `SWING` and `LOWER` share the exact same base angle ($q_0 = -0.8888\text{ rad}$), guaranteeing a pure vertical drop dead-center into the funnel tray.
9. **Physical Parcel Dynamic Modeling**:
   - Warehouse delivery parcels (`box_obstacle_1/2/3`) are configured with `<static>false</static>`, explicit physical mass ($0.4\text{ kg}$), and realistic inertial tensors, enabling natural contact dynamics, grasp compliance, and physical tray transfer.
10. **MoveIt 2 Allowed Collision Matrix (ACM) Exemption Tuning**:
    - Avoided false planning aborts by tuning ACM collision disable entries in `arm.srdf` for grounded pedestal links and configuring dynamic touch links in `planning_scene_manager.py`.\n

---

## Testing, Docker & CI

### Tests (headless, no simulator required)

```bash
# Fast suite: pure-logic units, geometry-vs-world drift guard, config checks
python3 -m pytest src/nav_nodes/test/ -v

# Full ament suite (requires sourced ROS 2 Jazzy)
colcon build --symlink-install --packages-select nav_nodes
colcon test --packages-select nav_nodes && colcon test-result --verbose
```

Test layers (fast → slow):
1. **Pure-logic units** — quaternion helper, dock-override parsing, TF stamp-monotonicity policy, mission state machine (retry/timeout/abort transitions) with stubbed action/service clients.
2. **Geometry-vs-world consistency** — parses `large_warehouse.sdf` and `warehouse.launch.py` and asserts the hardcoded constants in `station_arm_node.py` / `mission_node.py` still match the simulation within 1 mm. If you move a parcel, arm, or marker in the world, update the Python constants — CI will fail otherwise.
3. **Config validation** — `nav2_params.yaml` completeness + AMCL initial pose vs spawn pose, map YAML/PGM consistency, xacro expansion.
4. **ament lint + IK checks** — flake8/pep257/`test_station_ik.py` via `colcon test`.

### Docker

```bash
docker build --target dev -t robotics-ws:dev .
docker run -it --rm --net=host --ipc=host robotics-ws:dev   # sourced shell
# or:
docker compose run --rm dev
docker compose up sim                                       # headless sim stack
```

The image is multi-stage: a cached rosdep dependency layer, a colcon build layer, and a `dev` target with the lint toolchain.

### CI (GitHub Actions)

`.github/workflows/ci.yml` runs on every push/PR to `master`:
- **lint** — ruff + pylint
- **test** — `colcon build/test` for `nav_nodes` + the headless pytest suite in a `ros:jazzy-ros-base` container (no Gazebo)
- **docker** — builds the dev image as a full compile check of all packages

Local linting: `ruff check src/ && pylint --rcfile=.pylintrc src/nav_nodes/nav_nodes` (or install pre-commit: `pip install pre-commit && pre-commit install`).
