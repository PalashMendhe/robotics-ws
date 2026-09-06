# `src/robot_description/worlds/`

This directory contains the Gazebo Harmonic Simulation Description Format (SDF) world definitions.

---

## Files

### 1. `large_warehouse.sdf`
- **What it does**:
  - Primary simulation world for the autonomous warehouse environment.
  - **Warehouse Infrastructure**: Boundary walls ($16\text{ m} \times 16\text{ m}$), floor plane with realistic contact friction, and ambient lighting.
  - **Storage Shelving**: 17 heavy industrial shelf racks arranged into structured aisles.
  - **Delivery Boxes (`box_obstacle_1`, `box_obstacle_2`, `box_obstacle_3`)**:
    - Placed on the floor directly adjacent to Stations 1, 2, and 3 at $x=-4.20\text{ m}$, $y=0.6173 / 1.617 / 2.617\text{ m}$, $z=0.055\text{ m}$.
    - Configured with `<static>false</static>` and realistic physical properties ($0.4\text{ kg}$ mass, inertial tensor) so they can be physically grasped, lifted, and dropped into the AMR tray.
  - **Delivery Destination Markers**:
    - Destination 1: Circular green floor pad at `(4.4586, 4.3795)`.
    - Destination 2: Circular green floor pad at `(4.4590, -0.9218)`.
  - **Physics Engine**: Configured for Gazebo Harmonic ODE physics with step size $0.001\text{ s}$ and real-time update rate.
- **Requirement**: Primary simulation environment loaded by `warehouse.launch.py`.\n