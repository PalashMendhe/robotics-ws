# About me 
An upcoming pre-final year student in BIT Mesra, focusing on simulation-based autonomous navigation in ROS 2 and softwares related to it.

# ROS2 Robotics Workspace
Simulated a differential drive robot- full stack from urdf modelling to EKF fused localization and waypoint navigation with custom navigator. Also uses a 3 step state machine to navigate around target objects. 

## DEMO
[![Waypoint Navigation Demo](https://img.youtube.com/vi/j7rMp31hbr0/0.jpg)](https://www.youtube.com/watch?v=j7rMp31hbr0)
## System Overview
Simulation - Gazebo Harmonic with differential drive, LiDAR, IMU, and camera plugins

Localization - robot_localization EKF fusing wheel odometry and IMU

Navigation - Custom proportional controller navigating waypoints using filtered odometry

Perception - YOLOv8 perception pipeline to find and/or navigate around target objects.

## Packages
| Package | Description |
|---|---|
| robot_description | URDF + Gazebo simulation |
| navigation_stack | EKF sensor fusion |
| waypoint_navigator | Autonomous waypoint navigation |
| week1_nodes | ROS 2 communication patterns |
| Image_collection(robot_description) | Perception Pipeline|

## Tech Stack
ROS 2 Lyrical, Gazebo Harmonic, C++, Python, robot_localization EKF

## Quick Start
```bash
ros2 launch robot_description gazebo.launch.py
ros2 launch navigation_stack ekf.launch.py
ros2 run waypoint_navigator waypoint_navigator
```

## Technical Decisions
1. Custom waypont navigator built from scratch as the nav2 is not avaiiable in ROS2 lyrical (at the time of writing this). The proportional controller follows rotate then drive strategy and attained an accuracy < 0.01.
2. Used ```revolute``` joints with large limits instead ```continuous``` - as the Gazebo Harmonic's diff drive plugin requires this."
3. Added loggers on waypoint_navigator for easier debugging.
4. Custom trained dataset to detect red cubes and cylinder.
5. A 3 step state machine to search, approach the target object and navigate around it (some problems when multiple targets on sight). 
6. Uses float instead of tensor data to avoid type issues with ROS2.


