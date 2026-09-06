# `src/robotic_4dof_arm/scripts/`

This directory contains Python scripts for autonomous MoveIt 2 pick-and-place execution and RViz marker visualization.

---

## Files

### 1. `pick_and_place.py`
- **What it does**:
  - Executes full autonomous pick-and-place demonstration using MoveIt 2.
  - Initializes planning scene obstacles via `planning_scene_manager.py`.
  - Moves through canonical poses:
    1. Reaches down to floor parcel at $(-0.484, -0.0893, 0.070)$.
    2. Closes gripper and attaches box collision object to gripper prongs.
    3. Lifts parcel vertically ($z=0.450$) to clear AMR chassis.
    4. Swings arm base toward AMR tray at high clearance.
    5. Lowers forearm vertically onto AMR tray at $(-0.384, 0.300, 0.250)$.
    6. Opens gripper and detaches box on AMR tray.
    7. Returns to upright Home stance.
- **Requirement**: High-level demonstration script for MoveIt 2 manipulation.

---

### 2. `planning_scene_manager.py`
- **What it does**:
  - Interfaces with MoveIt 2 Planning Scene topic (`/planning_scene`).
  - Publishes static collision geometry:
    - Warehouse ground plane.
    - Docked AMR chassis box at $(-0.384, 0.300, 0.090)$.
    - Floor parcel box at $(-0.484, -0.0893, 0.055)$.
  - Handles dynamic `AttachedCollisionObject` operations when gripping and releasing boxes.
- **Requirement**: Manages collision objects to ensure collision-free MoveIt planning.

---

### 3. `world_publisher.py`
- **What it does**:
  - Publishes `visualization_msgs/MarkerArray` to `/world_markers` for RViz2.
  - Visualizes docked AMR chassis bounding box, target floor box, spawn marker rings, and destination dock rings.
- **Requirement**: Visual feedback in RViz2.\n