# `src/robot_description/launch/`

This directory provides the ROS 2 launch orchestration scripts for Gazebo simulation, Nav2 navigation, and autonomous multi-station mission execution.

---

## Files

### 1. `mission.launch.py`
- **What it does**:
  - Master full-bringup launch file for the autonomous random-station docking and delivery mission.
  - Sequentially brings up:
    1. `warehouse.launch.py`: Gazebo Harmonic world, AMR, 3 station arms, ros2_control managers, TF, and topic bridges.
    2. `nav2.launch.py`: Nav2 stack with AMCL localization configured on `large_warehouse.yaml`.
    3. `station_nodes`: Spawns 3 delayed `station_arm_node` instances (`station_1_arm_node`, `station_2_arm_node`, `station_3_arm_node`) serving `/station_N/dock_arm`.
- **Requirement**: Primary launch script used to start the entire automated warehouse mission system with a single command.

---

### 2. `warehouse.launch.py`
- **What it does**:
  - Core simulation bringup script.
  - Launches Gazebo Harmonic with the `large_warehouse.sdf` environment.
  - Spawns the AMR at the designated home dock pad `(-4.5, -4.5, 0.08, yaw=0.9)`.
  - Configures `robot_state_publisher` for the AMR.
  - Spawns all 3 station arms (`station_1_arm`, `station_2_arm`, `station_3_arm`) along the wall at $x=-4.684\text{ m}$, $y=0.528 / 1.528 / 2.528\text{ m}$, $z=0.03\text{ m}$, $yaw=\pi$.
  - Spawns `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` for each station namespace (`arm1`, `arm2`, `arm3`).
  - Launches `broadcaster_node` to broadcast `odom -> base_footprint` TF.
  - Establishes `ros_gz_bridge` parameter bridge for clock, cmd_vel, odom, scan, imu, joint states, and cameras.
- **Requirement**: Foundational simulation launcher providing the physical world, hardware interfaces, and sensor bridges.

---

### 3. `nav2.launch.py`
- **What it does**:
  - Launches the Navigation 2 (Nav2) stack including AMCL (Adaptive Monte Carlo Localization), Navfn Global Planner, Costmap2D, Controller Server (DWB / RotationShimController), and Behavior Tree Navigator.
  - Configures the default map to `src/robot_description/maps/large_warehouse.yaml`.
  - Ingests parameters from `src/robot_description/config/nav2_params.yaml`.
- **Requirement**: Required for global path planning, dynamic obstacle avoidance, and precise navigation between docking stations and destination markers.\n