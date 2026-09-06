# `src/robotic_4dof_arm/worlds/`

This directory contains the testing world SDF definition for standalone manipulator operations.

---

## Files

### 1. `warehouse.sdf`
- **What it does**:
  - Gazebo Harmonic simulation world model containing warehouse floor, lighting, delivery target box at $(-0.484, -0.0893, 0.055)$, and visual dock rings at $(-0.384, 0.300, 0.005)$.
- **Requirement**: Simulation world loaded by `robotic_4dof_arm/launch/gazebo.launch.py`.\n