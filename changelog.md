# Robotics Workspace Changelog & Architecture Roadmap

**Date:** 2026-09-04  
**Workspace:** `~/Desktop/robotics-ws`  
**Target:** Autonomous Warehouse AMR with Nav2, SLAM, Visual Workcells, and Teleportation

---

## 1. Current State of the Workspace

### 1.1 Packages & File Structure
* **`src/robot_description`**:
  * **Robot URDF (`urdf/robot.urdf.xacro`)**: Differential-drive mobile base equipped with:
    * 2D LiDAR (`lidar_link`, `/scan`) at $z \approx 0.13\text{ m}$.
    * Dual Cameras: Front camera (`camera_link_1`, tilted $30^\circ$ down) and Rear camera (`camera_link_2`, tilted $20^\circ$ down with $180^\circ$ yaw).
    * Dual-tier body structure: Base plate $\rightarrow$ Cover plate $\rightarrow$ 4 corner standoffs $\rightarrow$ Standoff floor with $30\text{ mm}$ height, $15^\circ$ outward-angled orange tray funnel walls to retain delivery boxes.
  * **Arm URDF (`urdf/arm.urdf.xacro`)**: 6-DOF robotic arm with revolute joints, prismatic two-prong gripper, and `gz_ros2_control` system plugin.
  * **Worlds**: `multiroom.sdf` ($3$ cramped rooms, $2.5\text{ m}$ high walls, narrow doorways, $2$ shelves with hardcoded boxes).
  * **Launch Files**: `gazebo.launch.py` (robot + bridges) and `arm_gazebo.launch.py` (spawns robot and two dynamic arm instances with joint trajectory controllers).
* **`src/nav_nodes`**:
  * `broadcaster_node.py`: Subscribes to `/model/my_robot/odometry` and publishes TF `odom -> base_footprint`.
  * `aruco_detector_node.py`: Subscribes to front and rear camera feeds, detects `DICT_4X4_50` markers (IDs 0 and 1) without OpenCV 4.6 parameter segfaults, and broadcasts TF transforms `camera_link_X -> aruco_marker_N_<front|rear>`.
  * `arm_controller_node.py`: Prototype trajectory node sending 6-joint trajectory goals to `/arm1/arm_controller/joint_trajectory`.
* **`src/navigation_stack`**:
  * Contains early configuration files: `config/slam_toolbox.yaml`, `config/ekf.yaml`, `launch/slam.launch.py`.

### 1.2 Identified Architectural Bottlenecks & Limitations
1. **Dynamic Arm Complexity**: Simulating two full 6-DOF arms with active physics joints and trajectory controllers caused frequent `controller_manager` namespace collisions, high CPU usage, and gripper physics instability (boxes slipping/dropping).
2. **Cramped Multiroom World**: The $2.5\text{ m}$ tall walls created obstructed camera angles and narrow doorways that are unrealistic for large-scale enterprise AMR workflows.
3. **Missing Autonomous Navigation Stack**: Navigation previously relied on manual teleoperation or prototype proportional controllers without dynamic obstacle avoidance, global costmaps, or recovery behaviors.
4. **Nav2 vs. Docking Conflict**: Standard Nav2 planners treat shelves and docking stations as obstacles, aborting when the robot gets too close.

---

## 2. Planned Changes & New Architecture

### 2.1 Environment Overhaul: `large_warehouse.sdf`
* **Scale**: Large-scale $25\text{ m} \times 20\text{ m}$ warehouse floor.
* **Perimeter Walls**: $0.9\text{ m}$ tall (waist-height), providing clean top-down visual clarity in Gazebo/RViz while staying well above the LiDAR scanning plane ($z \approx 0.13\text{ m}$).
* **Interior Layout**:
  * $3$ double-sided storage shelving aisles with wide $2.5\text{ m}$ transit corridors.
  * Structural support pillars, pallet stacks, and staging bins to provide rich 2D geometric features for laser scan-matching.
  * Designated **AMR Home / Charging Pad** at $(x = 2.0\text{ m}, y = 1.0\text{ m})$.
* **Station Workcells**:
  * **Station 1 (Inbound / Pickup Bay)** at $(x = 3.0\text{ m}, y = 3.0\text{ m})$:
    * Elevated industrial pedestal plinth ($0.3\text{ m} \times 0.3\text{ m} \times 0.15\text{ m}$).
    * Static visual 6-DOF arm mounted in an aesthetic resting pose.
    * ArUco Marker ID `0` mounted on the station facing the approach lane.
    * Initial spawn location for `box_1`.
  * **Station 2 (Outbound / Dropoff Bay)** at $(x = 21.0\text{ m}, y = 16.0\text{ m})$:
    * Elevated industrial pedestal plinth ($0.3\text{ m} \times 0.3\text{ m} \times 0.15\text{ m}$).
    * Static visual 6-DOF arm mounted in an aesthetic resting pose.
    * ArUco Marker ID `1` mounted on the station facing the approach lane.

### 2.2 Manipulation Simplification: Deterministic Teleportation
* Eliminate dynamic arm trajectory physics and `gz_ros2_control` overhead.
* When the AMR finishes its docking alignment via ArUco:
  * **Pickup**: Teleport `box_1` directly into the AMR's angled funnel tray using Gazebo's `/world/<world>/set_pose` service.
  * **Dropoff**: Teleport `box_1` from the AMR tray onto the Station 2 shelf.
* Result: 100% deterministic, zero physics jitter, zero dropped packages.

### 2.3 Navigation & Mapping Stack (Nav2 + SLAM Toolbox)
* **SLAM Mapping**: Run `async_slam_toolbox_node` while driving the AMR through warehouse aisles to create and save `large_warehouse.yaml` and `.pgm`.
* **Nav2 Integration**:
  * Global Planner: NavFn / Smac Planner 2D.
  * Local Controller: Regulated Pure Pursuit (RPP) / DWB tuned for diff-drive base.
  * Costmaps: Configured with obstacle and inflation layers with tuned footprint.
  * Localization: AMCL localized against the saved static map.
* **Two-Tier Navigation Architecture**:
  1. **Tier 1 (Nav2)**: Routes AMR from anywhere in the warehouse to an **Approach Waypoint** ($0.8\text{ m}$ outside the station costmap inflation layer).
  2. **Tier 2 (Docking Node)**: Bypasses Nav2 costmap once at the approach waypoint, executing closed-loop visual servoing to dock within $\pm 2\text{ cm}$ using the ArUco marker.

### 2.4 Mission Orchestration (`mission_orchestrator.py`)
A master coordinator executing the automated mission cycle:
1. Dispatch Nav2 to Station 1 Approach Waypoint.
2. Trigger ArUco Docking Node for terminal alignment.
3. Teleport `box_1` to AMR tray.
4. Dispatch Nav2 across the facility to Station 2 Approach Waypoint.
5. Trigger ArUco Docking Node for terminal alignment.
6. Teleport `box_1` to Station 2 shelf.
7. Return AMR to Home / Charging Pad.

---

## 3. Progress Tracking

- [x] Phase 0: Requirements analysis, risk identification, and roadmap approval.
- [x] Documentation: Created `changelog.md` tracking current vs. target architecture.
- [ ] Phase 1 (Day 1): Generate `large_warehouse.sdf` with 0.9m walls, storage aisles, and visual workcells.
- [ ] Phase 1 (Day 1): Update launch configurations and verify teleop navigation and sensor streams.
- [ ] Phase 2 (Day 2): Perform SLAM mapping and save `large_warehouse.yaml`.
- [ ] Phase 2 (Day 2): Configure and test Nav2 autonomous waypoint navigation.
- [ ] Phase 3 (Day 3): Implement Two-Tier ArUco docking node and `set_pose` box teleportation.
- [ ] Phase 3 (Day 3): Implement and demonstrate end-to-end `mission_orchestrator.py`.
