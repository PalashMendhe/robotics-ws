# ROS2 Robotics Workspace
Simulated a differential drive robot- full stack from urdf modelling to EKF fused localization and waypoint navigation with custom navigator.
## DEMO
https://github.com/user-attachments/assets/b433b780-af23-482c-bd82-a6b9960403b9
## System Overview
Simulation - Gazebo Harmonic with differential drive, LiDAR, IMU, and camera plugins

Localization - robot_localization EKF fusing wheel odometry and IMU

Navigation - Custom proportional controller navigating waypoints using filtered odometry
## Packages
| Package | Description |
|---|---|
| robot_description | URDF + Gazebo simulation |
| navigation_stack | EKF sensor fusion |
| waypoint_navigator | Autonomous waypoint navigation |
| week1_nodes | ROS 2 communication patterns |

## Tech Stack
ROS 2 Lyrical, Gazebo Harmonic, C++, Python, robot_localization EKF

## Quick Start
```bash
ros2 launch robot_description gazebo.launch.py
ros2 lauch navigation_stack ekf.launch.py
ros2 run waypoint_navigator waypoint_navigator
```

## Technical Decisions
1. Custom waypont navigator built from scratch as the nav2 is not avaiiable in ROS2 lyrical (at the time of writing this). The proportional controller follows rotate then drive strategy and attained an accuracy < 0.01.
2. Used ```revolute``` joints with large limits instead ```continuous``` - as the Gazebo Harmonic's diff drive plugin requires this."
3. Added loggers on waypoint_navigator for easier debugging. 
## Problems Faced and some Solutions
1. Rviz my_robot spawn issue - the robot was spawning in the enviornment but it was not visible. Most probably the device issue.
   
2. my_robot was unable to move and doing wheelie in the gazebo simulation.
   Reason - Centre of mass was at the back of bot and some bot spawning issues.
   Fix - Brought the centre of mass forward, reduced the wheel diameter, reduced the mass of the main body a little bit for even distibution and reduced the value of z-axis during spawn for proper footing of wheels.
   
3. my_robot was unable to move in square trail, which is still a visual problem.
   Reason - Robot was not properly aligning itself at each waypoint and high linear gains
   Fix - Trial & tested heading errors and corrected it same with linear gains.

