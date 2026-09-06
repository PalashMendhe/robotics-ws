# `src/arm_moveit_config/launch/`

This directory provides the launch automation for the MoveIt 2 motion planning pipeline.

---

## Files

### 1. `moveit.launch.py`
- **What it does**:
  - Assembles and launches the complete MoveIt 2 runtime:
    - Loads URDF description from `robotic_4dof_arm/urdf/arm.urdf.xacro`.
    - Loads SRDF semantic model from `arm_moveit_config/config/arm.srdf`.
    - Loads kinematics solver and joint limits configurations.
    - Starts the `move_group` node executing motion planning pipelines.
    - Launches RViz2 pre-configured with `moveit.rviz` if `use_rviz:=true`.
- **Requirement**: Primary launch script to run MoveIt 2 motion planning for the 6-DOF arm.\n