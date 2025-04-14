# ROS-2-Exercises

This repository contains my ROS 2 exercises and projects, created for learning purposes.

## my_ik_solver
This project implements a simple numerical inverse kinematics solver using the Gradient Method for the UR5 robot, based on its Denavit-Hartenberg (DH) parameters.

## basic_path_planning
This package implements a basic path planning and obstacle avoidance algorithm for controlling a turtle in the TurtleSim environment using ROS 2.

## my_vector_field
"The Vector Field method is a local path planning algorithm. I implemented it based on my lecture notes using C++ and integrated it into the Robotnik robot simulation (RobotnikAutomation/robotnik_simulation). Since the Robotnik simulation does not provide odometry by default, I also wrote an odometry publisher in C++. It is included in the same my_vector_field package as odom_publisher.cpp. Here's a GIF of the result:
![Simulation demo](media/Screencast from 2025-04-14 21-38-59 (online-video-cutter.com).gif)
