# `src/robotic_4dof_arm/config/`

This directory contains standalone controller configurations and RViz visualization layouts for the 6-DOF manipulator.

---

## Files

### 1. `controllers.yaml`
- **What it does**:
  - Configures `ros2_control` controller manager for standalone arm simulation.
  - Defines `joint_state_broadcaster`, `arm_controller` (JointTrajectoryController), and `gripper_controller`.
- **Requirement**: Required by `robotic_4dof_arm/launch/gazebo.launch.py`.

---

### 2. `display.rviz`
- **What it does**: RViz2 display configuration showing the arm links, TF coordinate frames, and interactive joint sliders.
- **Requirement**: Used by `robotic_4dof_arm/launch/display.launch.py`.\n