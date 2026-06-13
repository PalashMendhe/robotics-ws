# About me 
An upcoming pre-final year student in BIT Mesra, focusing on simulation-based autonomous navigation in ROS 2 and softwares related to it.

# ROS2 Robotics Workspace
Simulated a differential drive robot having a 4+1 dof robotic arm attached on top of it - Full stack from urdf modelling, EKF sensor fusion, custom waypoint navigator, yolo v8 perceptio and unified keyboard teleop - arm control and teleop. 

## DEMO

### Waypoint Navigation Demo 

[![Waypoint Navigation Demo](https://img.youtube.com/vi/j7rMp31hbr0/0.jpg)](https://www.youtube.com/watch?v=j7rMp31hbr0)

### Differential Drive and Arm Control Demo

[![Differential drive and arm control Demo](https://img.youtube.com/vi/XCMsv4JSbp4/0.jpg)](https://www.youtube.com/watch?v=XCMsv4JSbp4) 
## System Overview
Simulation - Gazebo Harmonic with differential drive, LiDAR, IMU, and camera plugins.

Localization - robot_localization EKF fusing wheel odometry and IMU.

Navigation - Custom proportional controller navigating waypoints using filtered odometry.

Perception - YOLOv8 perception pipeline to find and/or navigate around target objects.

Unified Controller - Keyboard controlled arm and differential drive.

Inversed Kinematics solver - To navigate the arm to specified coordinates with a single line command.

PID integration - To negate the effect of gravity.


## Packages
| Package | Description |
|---|---|
| arm_description | 4+1 dof robotic arm |
| arm_teleop | arm and differential drive controller |
| Image_collection(robot_description) | Perception Pipeline|
| robot_description | URDF + Gazebo simulation |
| navigation_stack | EKF sensor fusion |
| waypoint_navigator | Autonomous waypoint navigation |
| week1_nodes | ROS 2 communication patterns |


## Tech Stack
ROS 2 Lyrical, Gazebo Harmonic, C++, Python, robot_localization EKF 

## Quick Start
***For Waypoint Navigation***
```bash
ros2 launch robot_description gazebo.launch.py
ros2 launch navigation_stack ekf.launch.py
ros2 run waypoint_navigator waypoint_navigator
```

***For Arm control and differential drive control***
```bash
ros2 launch robot_description gazebo.launch.py
ros2 run arm_teleop arm_keyboard
```

***For specific Coordinate (with example)***
```bash
ros2 launch robot_description gazebo.launch.py
ros2 run arm_teleop arm_keyboard
ros2 service call /get_arm_ik arm_description/srv/GetArmIK "{x: 0.2, y: 0.0, z: 0.3}"
```

## Technical Decisions
1. Custom waypont navigator built from scratch as the nav2 is not avaiiable in ROS2 lyrical (at the time of writing this). The proportional controller follows rotate then drive strategy and attained an accuracy < 0.01.
2. Used ```revolute``` joints with large limits instead ```continuous``` - as the Gazebo Harmonic's diff drive plugin requires this."
3. Added loggers on waypoint_navigator for easier debugging.
4. Custom trained dataset to detect red cubes and cylinder.
5. A 3 step state machine to search, approach the target object and navigate around it (some problems when multiple targets on sight). 
6. Uses float instead of tensor data to avoid type issues with ROS2.
7. Different worlds for waypoint navigation and, arm control and differential drive.
8. Added PID to robotic arm to stabalize it on robot.
9. A custom arm keyboard to control the whole bot
10. Used inverse kinematics to reach a specific coordinate.



