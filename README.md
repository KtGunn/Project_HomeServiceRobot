# Project_HomeServiceRobot

## Overview

This is the final project of Udacity's Robotics Software Nanodegree program (RSND). The project's objective is to create a robotic operations simulation. The operation will command a robot to pickup and carry and object from one location to another. The object will be simulated through visible markers placed in the map at the pick up and drop off locations.

### Tasks

A sequence of steps building up to the final goal were defined:

- Create a simulation environment in Gazebo.
- Create a shell script to launch all nodes required to map the environment. The gmapping SLAM package is used and the robot is manually driven using tele operation.
- Create a shell script to launch all nodes required to navigate the robot through the map and environment, using rviz to specify navigation goals.
- Finally, create a shell script to launch all nodes required to perform the simulation. The simulation occurs automatically with no input from the user.

### Packages

Several packages are used for the project:
- slam_gmapping
- amcl
- turtlebot
- turtlebot_simulator
- turtlebot_interactions

### Host System

The turtlebot robot was prescribed for this project but, as explained later, a robot created in prior projects was used also. The project was delveloped under Ubuntu 16.04.7 using ROS distribution Kinetic.
