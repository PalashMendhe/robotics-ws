# Engineering Bug Log & Root Cause Analysis

This document compiles the major engineering bugs, edge cases, and simulation anomalies encountered during the development of this ROS 2 autonomous warehouse logistics system, along with root cause investigations, mathematical derivations, and technical resolutions.

---

## 1. Skid-Steer In-Place Pivot Stall in Gazebo Harmonic ODE
- **Symptom**: When commanding pure rotation ($v_x = 0.0\text{ m/s}, \omega_z = 1.2\text{ rad/s}$), the 4-wheeled AMR stalled at $89^\circ$, spinning wheels furiously while body angular velocity collapsed to $\omega_z = 0.014\text{ rad/s}$.
- **Root Cause**: In Gazebo Harmonic's Open Dynamics Engine (ODE), cylinder-to-plane wheel contact without an explicit friction direction applies isotropic friction (equal resistance in all contact plane directions). For a 4-fixed-wheel skid-steer chassis, yaw rotation requires the wheels to slip laterally while rolling longitudinally. With high isotropic friction ($\mu_1 = \mu_2 = 1.1$), lateral ground contact forces counteracted motor drive torque, physically locking the chassis in place.
- **Resolution**:
  - In `src/robot_description/urdf/robot.urdf.xacro`, added explicit `<fdir1>1 0 0</fdir1>` tags to all 4 wheel links to align primary friction strictly along the wheel's longitudinal rolling direction.
  - Decoupled lateral friction by setting `<mu2>0.05</mu2>` and `<dynamics damping="0.001" friction="0.0"/>`.
- **Verification**:
  - Commanded: $v_x = 0.400\text{ m/s}, \omega_z = 0.000\text{ rad/s} \rightarrow$ Measured: $v_x = 0.400\text{ m/s}, \omega_z = 0.000\text{ rad/s}$ (Zero drift).
  - Commanded: $v_x = 0.000\text{ m/s}, \omega_z = 1.200\text{ rad/s} \rightarrow$ Measured: $\omega_z = 1.1997\text{ rad/s}$ (0.02% error).

---

## 2. Global Planner Path Shortcutting Through Hollow Shelves
- **Symptom**: The Nav2 global planner (Navfn) regularly planned diagonal paths cutting directly through industrial storage shelves.
- **Root Cause**: The 2D map originally produced by naive raycasting/SLAM mapped only the thin outer perimeter walls and legs of the shelves. The interior volume of the shelves was marked as `254` (unknown / free space). Because Navfn seeks the shortest Euclidean path, it routed straight through the empty shelf interiors.
- **Resolution**:
  - Developed an exact geometric rasterizer that computed 2D bounding footprints for all 17 warehouse storage shelves in `large_warehouse.sdf`.
  - Rasterized the entire 2D area of each shelf as 100% solid occupied cells (`0` = black) in `large_warehouse.pgm` and `large_warehouse.yaml`.
- **Verification**: Navfn and Costmap2D now treat all shelves as solid obstacles with inflation boundaries, forcing paths strictly through warehouse aisles.

---

## 3. Dynamic TF Tree Contention (`robot_localization` vs `broadcaster_node`)
- **Symptom**: Robot pose in RViz2 flickered violently between two locations; AMCL particle cloud dispersed across the map; Nav2 aborted navigation with `TF_OLD_DATA` and `ExtrapolationException` errors.
- **Root Cause**: `ekf.launch.py` ran an EKF filter broadcasting `odom -> base_link`, while `broadcaster_node` simultaneously broadcast `odom -> base_footprint`. Having two nodes publishing conflicting parent/child transforms in the same TF tree corrupted the coordinate chain.
- **Resolution**:
  - Excluded `ekf.launch.py` from the production launch sequence (`mission.launch.py`).
  - Standardized on `broadcaster_node.py` subscribing directly to `/model/my_robot/odometry` from Gazebo and publishing a single authoritative `odom -> base_footprint` transform with strictly monotonic timestamps (`msg_nanos > self.last_stamp_nanos`).
- **Verification**: Zero transform jitter, zero TF dropped frames, and stable AMCL pose tracking across full warehouse navigation legs.

---

## 4. Floating Station Arms & Contorted Spawn Stances
- **Symptom**: Upon launch, station arms spawned floating in mid-air at $z = 0.60\text{ m}$. Furthermore, arms violently snapped and contorted into strange configurations upon controller activation.
- **Root Cause**:
  1. Spawner coordinates in `warehouse.launch.py` had $z = 0.60\text{ m}$ left over from legacy tabletop setups.
  2. The `<ros2_control>` state interfaces in `arm.urdf.xacro` had initial parameters hardcoded to legacy angles (`upper_arm_joint=1.57`, `forearm_joint=-1.57`, `wrist_joint=-1.57`). With modern zero-origins, this forced links to swing into unnatural horizontal extensions.
- **Resolution**:
  - Welded grounded pedestal links ($0.30\text{ m} \times 0.30\text{ m} \times 0.03\text{ m}$) flush with the warehouse floor at $z = 0.03\text{ m}$.
  - Set all `<ros2_control>` `<param name="initial_value">0.0</param>` across all 6 revolute joints, creating a safe upright neutral stance (`[0, 0, 0, 0, 0, 0]`).
  - Added joint dynamics damping ($5.0\text{ Nms/rad}$) and friction ($1.0\text{ Nm}$) to prevent gravity droop.
- **Verification**: Station arms spawn perfectly grounded on the floor, holding upright stances with zero joint drift.

---

## 5. Manipulator Ground Collision & Inverse Kinematics Singularities
- **Symptom**: When attempting to pick parcels from the floor ($z = 0.055\text{ m}$), the arm elbow slammed below the floor plane ($z < 0$), causing Gazebo physics engine collisions and joint controller aborts.
- **Root Cause**: Legacy analytic IK solver `_ik()` assumed the arm base zero-pose pointed along $+X$ and selected an elbow-down configuration. From a floor-level pedestal ($z = 0.03\text{ m}$), an elbow-down pose drives the elbow deep underground.
- **Resolution**:
  - Formulated closed-form analytical inverse kinematics enforcing an **elbow-up configuration**.
  - In elbow-up stance, the elbow remains elevated at $z_{elbow} > 0.45\text{ m}$ and wrist stays elevated at $z_{wrist} > 0.45\text{ m}$ throughout all pick and place operations.
  - Enforced strict joint limit bounds: all 6 revolute joints strictly satisfy $[-3.14, 3.14]$ radians.
- **Verification**: Verified across all 3 stations with unit test `test_station_ik.py` (`test_waypoints_within_reach` and `test_waypoints_not_clamped` passing 100%).

---

## 6. Unmovable Delivery Parcels in Gazebo Simulation
- **Symptom**: Station arm closed its parallel prongs on the delivery box, but upon lifting, the box remained frozen in mid-air on the floor while the gripper slipped off.
- **Root Cause**: In `large_warehouse.sdf`, the parcel models (`box_obstacle_1`, `box_obstacle_2`, `box_obstacle_3`) were defined with `<static>true</static>`. In Gazebo Harmonic, static models are treated as infinite-mass static terrain and cannot be moved by contact forces.
- **Resolution**:
  - Changed `<static>false</static>`.
  - Added physical mass ($0.4\text{ kg}$) and appropriate box inertia tensors:
    ```xml
    <inertial>
      <mass>0.4</mass>
      <inertia>
        <ixx>0.000667</ixx><iyy>0.000667</iyy><izz>0.000667</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
      </inertia>
    </inertial>
    ```
- **Verification**: Gripper firmly grasps parcel with clamping joints `[0.018, -0.018]`, lifting it cleanly off the floor.

---

## 7. Chassis Scraping & Funnel Rim Collision During Parcel Transfer
- **Symptom**: When swinging the gripped parcel from floor grasp to AMR tray, the box clipped the front of the robot chassis, and the gripper prongs scraped the top rim of the tray funnel ($z = 0.237\text{ m}$).
- **Root Cause**: Interpolating simultaneously along XYZ and base yaw from floor pick ($z = 0.10\text{ m}$) to tray place passed through a diagonal trajectory that climbed in altitude too late. Furthermore, because the $0.10\text{ m}$ box hangs $50\text{ mm}$ below the gripper TCP, holding TCP near tray height placed the bottom of the parcel below the lip.
- **Resolution**:
  - Implemented a dedicated 2-stage vertical clearance trajectory:
    1. **Vertical `LIFT`**: Pure vertical ascent at parcel coordinates ($x = -4.20\text{ m}, y = 0.6173\text{ m}$) with zero base rotation to TCP $z = 0.480\text{ m}$ (box bottom at $z = 0.430\text{ m}$, providing **$+193\text{ mm}$ clean clearance** above the AMR rim).
    2. **High-Altitude `SWING`**: Base rotates to AMR heading while holding the box high at $z = 0.480\text{ m}$.
    3. **Pure Vertical `LOWER`**: Forearm lowers vertically into the funnel at $z = 0.280\text{ m}$ with zero base rotation.
- **Verification**: Clean $+193\text{ mm}$ air gap verified across all stations; zero contact during swing.

---

## 8. In-Transit Parcel Sliding & Cornering Loss
- **Symptom**: AMR accelerated or turned sharply, causing the carried box to slide across the robot top and fall onto the warehouse floor.
- **Root Cause**: Flat top surface provided zero mechanical retention; isotropic jerk during rotational acceleration generated inertial forces exceeding surface friction.
- **Resolution**:
  - Designed an integrated mechanical funnel tray mounted on top of the standoff floor with four $4.5\text{ cm}$ angled walls ($15^\circ$ outward tilt). Dropped boxes naturally settle into the center.
  - Configured Nav2 `RotationShimController` (`rotate_to_heading_angular_vel: 1.8`, `angular_dist_threshold: 0.785`) to pivot in place smoothly before accelerating along straight path segments.
  - Added `velocity_smoother` with gentle linear acceleration limit ($2.0\text{ m/s}^2$).
- **Verification**: Parcels remain fully seated inside the tray during emergency stops, $180^\circ$ pivots, and high-speed transit to destinations.

---

## 9. Docking Stand-Off Misalignment & Funnel Edge Drops
- **Symptom**: AMR stopped slightly short of the station arm ($x = -4.050\text{ m}$ then $-4.200\text{ m}$), causing the arm to reach near its maximum extension, where small trajectory errors resulted in the box striking the outer funnel lip.
- **Root Cause**: The docking standoff coordinate was overly conservative along $X$, placing the AMR tray at a larger radial distance ($r = 0.569\text{ m}$) than the pick position ($r = 0.492\text{ m}$).
- **Resolution**:
  - Advanced docking position by $0.100\text{ m}$ closer to the arm: `DOCK_X = -4.300\text{ m}`.
  - Bumper clearance from AMR front ($x = -4.475\text{ m}$) to arm pedestal ($x = -4.534\text{ m}$) is $5.9\text{ cm}$, completely collision-free.
  - Radial reach to tray now equals $r = \sqrt{0.384^2 + 0.300^2} = 0.487\text{ m}$, nearly identical to pick radius ($0.492\text{ m}$).
  - Set `SWING` and `LOWER` to share the exact same base angle ($q_0 = -0.8888\text{ rad}$), guaranteeing pure vertical descent into the funnel center.
- **Verification**: Parcel drops dead-center into the tray with $0\text{ mm}$ lateral drift across all test runs.

---

## 10. MoveIt 2 Semantic Collision Aborts & Coordinate Discrepancies
- **Symptom**: MoveIt 2 motion planning aborted with `INVALID_MOTION_PLAN` and `COLLISION` errors when executing pick-and-place trajectories on the 6-DOF arm.
- **Root Cause**:
  1. The Allowed Collision Matrix (ACM) in `arm.srdf` lacked disable entries for the newly added grounded `pedestal` link.
  2. Legacy scripts in `robotic_4dof_arm` referenced hardcoded tabletop obstacle coordinates instead of floor parcel coordinates.
- **Resolution**:
  - Updated `src/arm_moveit_config/config/arm.srdf` with ACM disable entries for `pedestal` with `world`, `arm_base_link`, and `upper_arm_link`.
  - Synchronized `pick_and_place.py` and `planning_scene_manager.py` with the production floor parcel coordinates ($x = -0.484\text{ m}, y = -0.0893\text{ m}$) and AMR tray coordinates ($x = -0.384\text{ m}, y = 0.300\text{ m}$).
- **Verification**: MoveIt 2 plans and executes pick-and-place trajectories collision-free in RViz2 and Gazebo.\n