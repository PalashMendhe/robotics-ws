# `src/nav_nodes/test/`

This directory contains automated unit tests and linter tests verifying kinematics, workspace geometry, and coding standards.

---

## Files

### 1. `test_station_ik.py`
- **What it does**:
  - Comprehensive Pytest test suite containing 7 unit tests:
    1. `test_waypoints_within_reach`: Confirms all 6 arm joints across all waypoints stay strictly within $[-3.14, 3.14]$ radians.
    2. `test_waypoints_not_clamped`: Asserts all arm waypoints have real elbow flexion ($\\ge 0.05\text{ rad}$), ensuring targets are within reach and not singular/clamped.
    3. `test_pick_and_place_reach`: Verifies all station TCP targets fall within the valid arm reach envelope ($0.30\text{ m} < r < 0.80\text{ m}$).
    4. `test_dock_pose_clears_parcel`: Verifies the docked AMR chassis maintains $> 0.21\text{ m}$ lateral clearance from the floor parcel to prevent clipping.
    5. `test_gripper_body_clears_parcel`: Asserts the gripper body bottom clears the top of the floor parcel by at least $25\text{ mm}$ during descent.
    6. `test_dock_poses_single_source_of_truth`: Verifies exact correspondence between `STATION_DOCK_POSES` and station geometry (`DOCK_X == ARM_X + 0.384`).
    7. `test_lift_and_swing_clearance_above_amr`: Validates that parcel bottom remains $> 190\text{ mm}$ above the AMR tray rim during vertical `LIFT` and horizontal `SWING`.
- **Requirement**: Run with `pytest src/nav_nodes/test/test_station_ik.py` to prevent regressions in robot kinematics and dock poses.

---

### 2. Linter Tests
- `test_copyright.py`: Checks copyright declaration across source files.
- `test_flake8.py`: Validates PEP 8 styling and syntax.
- `test_pep257.py`: Validates Python docstring conventions.\n