# `robotic_4dof_arm` Package

The `robotic_4dof_arm` package provides the standalone 6-DOF robotic manipulator simulation, URDF model, controllers, and MoveIt-driven pick-and-place demonstration scripts.

---

## Package Structure

```
robotic_4dof_arm/
├── CMakeLists.txt        # ament_cmake build configuration
├── package.xml           # Package metadata and dependencies
├── config/               # Standalone controllers and RViz layout
├── launch/               # Gazebo and display launch files
├── scripts/              # Autonomous pick-and-place & scene management
├── urdf/                 # 6-DOF arm URDF/Xacro model
└── worlds/               # Standalone testing world
```

---

## Subdirectories

- [`config/`](config/README.md): Standalone controller configurations.
- [`launch/`](launch/README.md): Display and standalone Gazebo launch scripts.
- [`scripts/`](scripts/README.md): Autonomous pick-and-place scripts.
- [`urdf/`](urdf/README.md): Standalone arm Xacro model.
- [`worlds/`](worlds/README.md): Testing world description.\n