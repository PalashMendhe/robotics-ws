# `arm_moveit_config` Package

The `arm_moveit_config` package contains the MoveIt 2 semantic robot configuration, kinematics parameters, joint limits, planning pipelines (OMPL), and launch orchestration for the 6-DOF robotic manipulator.

---

## Package Structure

```
arm_moveit_config/
├── CMakeLists.txt        # Build and installation rules (ament_cmake)
├── package.xml           # Dependencies on MoveIt 2, rclcpp, srdfdom
├── config/               # MoveIt semantic & kinematics configuration
│   ├── arm.srdf          # Planning groups, named states, collision matrix
│   ├── joint_limits.yaml # Velocity/acceleration limits
│   ├── kinematics.yaml   # KDL solver configuration
│   ├── moveit_controllers.yaml # ros2_control controller bindings
│   ├── ompl_planning.yaml# OMPL motion planning algorithms
│   └── moveit.rviz       # Pre-configured RViz2 display
└── launch/
    └── moveit.launch.py  # MoveIt 2 bringup launch file
```

---

## Subdirectories

- [`config/`](config/README.md): Detailed documentation of SRDF, joint limits, kinematics, and OMPL parameters.
- [`launch/`](launch/README.md): Explanation of `moveit.launch.py`.\n