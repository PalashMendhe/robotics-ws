# `src/robotic_4dof_arm/urdf/`

This directory contains the standalone URDF/Xacro model for the 6-DOF robotic manipulator.

---

## Files

### 1. `arm.urdf.xacro`
- **What it does**:
  - Complete kinematic model of the 6-DOF robotic manipulator.
  - Includes grounded pedestal ($0.30\text{ m} \times 0.30\text{ m} \times 0.03\text{ m}$), 6 revolute joints strictly within $[-3.14, 3.14]$, joint damping dynamics, parallel-jaw prismatic gripper, and `<ros2_control>` hardware interface tags.
- **Requirement**: Core robot description for the `robotic_4dof_arm` and `arm_moveit_config` packages.\n