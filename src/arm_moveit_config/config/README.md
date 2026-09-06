# `src/arm_moveit_config/config/`

This directory provides MoveIt 2 configuration files defining planning groups, kinematics plugins, joint limits, trajectory controllers, and collision exemptions.

---

## Files

### 1. `arm.srdf` (Semantic Robot Description Format)
- **What it does**:
  - **Planning Groups**:
    - `arm`: Kinematic chain from `arm_base_link` to `gripper_base_link`.
    - `gripper`: End effector group containing `right_prong_joint` and `left_prong_joint`.
  - **Named Group States**:
    - `home`: Upright neutral posture (`[0, 0, 0, 0, 0, 0]`).
    - `pick_grasp`: Reaching down to floor parcel (`[-0.0409, -0.3555, -0.2051, 0.2248, 0, 0]`).
    - `lift`: Vertical parcel lift above AMR rim (`[-0.0409, -0.2951, 0.7249, 1.2153, 0, 0]`).
    - `swing`: Elevated rotation facing AMR tray (`[-0.8888, -0.2809, 0.7059, 1.2105, 0, 0]`).
    - `lower`: Vertical descent into AMR tray (`[-0.8888, -0.2030, 0.0846, 0.6671, 0, 0]`).
    - `open` / `closed`: Gripper open (`[0.06, -0.06]`) and closed (`[0.018, -0.018]`).
  - **Allowed Collision Matrix (ACM)**: Disables collision checking between statically welded or adjacent links (e.g. `pedestal` with `world` and `arm_base_link`).
- **Requirement**: Mandatory MoveIt 2 semantic file defining robot planning capabilities.

---

### 2. `joint_limits.yaml`
- **What it does**: Defines maximum joint velocity and acceleration bounds for all 6 arm joints and 2 gripper prongs to ensure safe, physically realistic trajectory generation.
- **Requirement**: Read by MoveIt trajectory generation algorithms.

---

### 3. `kinematics.yaml`
- **What it does**: Configures the inverse kinematics solver plugin (`kdl_kinematics_plugin/KDLKinematicsPlugin`) for the `arm` group with $0.005\text{ s}$ timeout and $0.0001$ search resolution.
- **Requirement**: Required by MoveIt to compute inverse kinematics for Cartesian goals.

---

### 4. `moveit_controllers.yaml`
- **What it does**: Configures `MoveItSimpleControllerManager` to dispatch planned trajectories to `ros2_control` action servers:
  - `arm_controller`: Action namespace `arm_controller/follow_joint_trajectory`.
  - `gripper_controller`: Action namespace `gripper_controller/follow_joint_trajectory`.
- **Requirement**: Required to execute MoveIt plans on simulated or real hardware.

---

### 5. `ompl_planning.yaml`
- **What it does**: Configures OMPL motion planners (RRTConnect, RRTstar, BKPIECE, EST, PRM) for the `arm` group.
- **Requirement**: Provides sampling-based path planning algorithms.

---

### 6. `moveit.rviz`
- **What it does**: Pre-saved RViz2 layout with `MotionPlanning` display, interactive 6-DOF markers, scene collision objects, and robot visual links enabled.
- **Requirement**: Visual interface for interactive planning in RViz2.\n