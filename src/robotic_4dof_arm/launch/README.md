# `src/robotic_4dof_arm/launch/`

This directory provides launch automation for standalone arm simulation and URDF inspection.

---

## Files

### 1. `gazebo.launch.py`
- **What it does**:
  - Starts Gazebo Harmonic with `warehouse.sdf`.
  - Spawns the grounded 6-DOF arm at origin `(0, 0, 0)`.
  - Starts `robot_state_publisher` and activates `ros2_control` controllers.
- **Requirement**: Primary launch script for standalone arm simulation.

---

### 2. `display.launch.py`
- **What it does**:
  - Launches `joint_state_publisher_gui` and `robot_state_publisher` with RViz2.
- **Requirement**: Used to inspect joint motion and limit ranges without launching Gazebo physics.\n