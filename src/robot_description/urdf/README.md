# `src/robot_description/urdf/`

This directory houses the complete kinematic and dynamic robot definitions in URDF/Xacro for both the Autonomous Mobile Robot (AMR) and the 6-DOF warehouse station arm.

---

## Files

### 1. `robot.urdf.xacro`
- **What it does**:
  - Defines the 4-wheeled differential drive Autonomous Mobile Robot (AMR).
  - **Chassis Dimensions & Inertia**: Main body box of size $0.35\text{ m} \times 0.25\text{ m} \times 0.07\text{ m}$ with $5.0\text{ kg}$ mass.
  - **Wheel Dynamics & Friction**: 4 fixed cylindrical wheels ($r=0.05\text{ m}, w=0.04\text{ m}$). Uses tuned ODE friction parameters (`<fdir1>1 0 0</fdir1>` and `<mu2>0.05</mu2>`) to separate rolling from sliding friction. This resolves skid-steer wheel scrubbing and enables true in-place pivoting without lateral drift.
  - **Sensor Payload**:
    - 2D Planar LiDAR (`/scan`) mounted on the top cover plate ($0.25\text{ m} - 10.0\text{ m}$ range).
    - Dual Depth Cameras: `/camera/front/` and `/camera/rear/` with image and camera info streams.
    - 6-axis IMU (`/imu`) publishing linear acceleration and angular velocity.
  - **Funnel Parcel Tray**: Standoff plate equipped with 4 angled perimeter lip walls ($15^\circ$ outward tilt, $4.5\text{ cm}$ height). Drops into this funnel automatically self-center the parcel, and the walls mechanically retain the box via pure physics during transit.
  - **Gazebo Plugins**: Integrates `gz::sim::systems::DiffDrive`, `gz::sim::systems::OdometryPublisher` (publishes true 2D odometry to `/model/my_robot/odometry`), and `gz::sim::systems::Sensors`.
- **Requirement**: Core mobile robot description. Required for AMR simulation, sensor streams, TF tree generation, and physical parcel retention.

---

### 2. `arm.urdf.xacro`
- **What it does**:
  - Models the 6-DOF robotic manipulator mounted at each of the 3 warehouse pick-and-place stations.
  - **Pedestal**: Grounded mounting plate ($0.30\text{ m} \times 0.30\text{ m} \times 0.03\text{ m}$) fixed flush to the warehouse floor at $z=0.03\text{ m}$.
  - **Kinematic Chain (6 Revolute Joints)**:
    1. `arm_base_joint`: Azimuth rotation around $Z$ axis.
    2. `upper_arm_joint`: Shoulder pitch around $Y$ axis (damping $5.0\text{ Nms/rad}$, friction $1.0\text{ Nm}$).
    3. `forearm_joint`: Elbow pitch around $Y$ axis (damping $5.0\text{ Nms/rad}$, friction $1.0\text{ Nm}$).
    4. `wrist_joint`: Wrist pitch around $Z$ axis (damping $2.0\text{ Nms/rad}$, friction $0.5\text{ Nm}$).
    5. `gripper_baseTOwrist_joint`: Wrist rotation around $X$ axis.
    6. `gripper_base_joint`: Tool roll around $Z$ axis.
    - All 6 revolute joints strictly operate within $[-3.14, 3.14]$ radians.
  - **Parallel Jaw Gripper**: Two prismatic prongs (`right_prong_joint`, `left_prong_joint`) providing up to $0.12\text{ m}$ opening width to easily clear the $0.10\text{ m}$ parcel.
  - **`<ros2_control>` Integration**: Uses `gz_ros2_control/GazeboSimSystem` hardware plugin with position command and state interfaces initialized cleanly to `0.0`.
- **Requirement**: Used by `warehouse.launch.py` to instantiate and control `arm1`, `arm2`, and `arm3` via `ros2_control` trajectory controllers.\n