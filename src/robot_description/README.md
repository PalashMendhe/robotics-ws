# `robot_description` Package

The `robot_description` package serves as the foundational package for the autonomous warehouse environment. It encapsulates the full URDF robot models (AMR chassis and 6-DOF station arm), Gazebo Harmonic world definitions, sensor configurations, Nav2 obstacle maps, ROS 2 control parameters, and top-level mission bringup launch scripts.

---

## Package Structure

```
robot_description/
├── CMakeLists.txt        # Build & installation rules (ament_cmake)
├── package.xml           # Package manifest and ROS 2 dependencies
├── config/               # Controller and Nav2 configuration YAMLs
├── launch/               # ROS 2 launch files (warehouse, Nav2, mission)
├── maps/                 # Solid rasterized warehouse map (.yaml / .pgm)
├── urdf/                 # Xacro kinematic/dynamic models for AMR and Arm
└── worlds/               # Gazebo Harmonic SDF world models
```

---

## Files in Package Root

### 1. `CMakeLists.txt`
- **Role**: Defines package build and install directives using `ament_cmake`.
- **Installed Directories**: `urdf`, `launch`, `config`, `worlds`, `maps`.
- **Requirement**: Required by `colcon build` to export package resources to the ROS 2 workspace `install/share/robot_description` directory.

### 2. `package.xml`
- **Role**: ROS 2 package manifest specifying format 3 metadata.
- **Dependencies**: `ament_cmake`, `robot_state_publisher`, `xacro`, `ros_gz_sim`, `ros_gz_bridge`.
- **Requirement**: Identifies the package within the ROS 2 ecosystem and resolves build/runtime dependencies.

---

## Subdirectories

- [`urdf/`](urdf/README.md): Contains `robot.urdf.xacro` (AMR) and `arm.urdf.xacro` (6-DOF station arm).
- [`launch/`](launch/README.md): Contains `mission.launch.py`, `warehouse.launch.py`, and `nav2.launch.py`.
- [`config/`](config/README.md): Contains `nav2_params.yaml`, `arm_controllers.yaml`, and `joint_bridge.yaml`.
- [`maps/`](maps/README.md): Contains `large_warehouse.yaml` and `large_warehouse.pgm`.
- [`worlds/`](worlds/README.md): Contains `large_warehouse.sdf`.\n