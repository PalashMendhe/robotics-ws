# `src/robot_description/config/`

This directory holds configuration parameter files for Nav2 autonomous navigation, ros2_control joint controllers, and topic bridges.

---

## Files

### 1. `nav2_params.yaml`
- **What it does**:
  - Complete configuration for all Navigation 2 lifecycle nodes:
    - **AMCL**: Initialized with spawn pose `(-4.5, -4.5, 0.0, yaw=0.9)` matching the warehouse pad; laser scan likelihood model; particle filter count (500-2000).
    - **Global Costmap & Local Costmap**: Layered with static obstacle layer (solid shelf footprints), inflation layer, and sensor obstacle layer.
    - **Controller Server**: Configured with `RotationShimController` for smooth in-place rotation before driving, preventing wheel slip and parcel shifting.
    - **Velocity Smoother**: Enforces acceleration limits ($2.0\text{ m/s}^2$) and deceleration limits ($-2.0\text{ m/s}^2$) for gentle transport.
    - **Goal Checker**: Checks arrival tolerance at $\pm 0.08\text{ m}$ xy and $\pm 0.15\text{ rad}$ yaw.
- **Requirement**: Mandatory configuration defining robot navigation behavior and path execution.

---

### 2. `arm_controllers.yaml`
- **What it does**:
  - Configuration for the `controller_manager` running inside each station arm namespace (`arm1`, `arm2`, `arm3`).
  - Defines:
    - `joint_state_broadcaster`: Publishes joint states to `/<arm_ns>/joint_states`.
    - `arm_controller`: `JointTrajectoryController` governing the 6 revolute joints (`arm_base_joint`, `upper_arm_joint`, `forearm_joint`, `wrist_joint`, `gripper_baseTOwrist_joint`, `gripper_base_joint`).
    - `gripper_controller`: `JointTrajectoryController` governing the prismatic gripper prongs (`right_prong_joint`, `left_prong_joint`).
- **Requirement**: Mandatory for `ros2_control` trajectory execution across all three station manipulators.

---

### 3. `joint_bridge.yaml`
- **What it does**:
  - Defines direct topic bridges between ROS 2 (`std_msgs/Float64`) and Gazebo (`gz.msgs.Double`) for individual joint position commands (`/arm_base_joint/cmd_pos`, etc.).
- **Requirement**: Preserved for single-robot prototype debugging and direct topic-level joint teleoperation.\n