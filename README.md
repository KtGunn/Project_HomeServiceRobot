# Project_HomeServiceRobot

## Overview

This is the final project of Udacity's Robotics Software Nanodegree program (RSND). The project's objective is to create a robotic operations simulation. The operation will command a robot to pickup an carry and object from one location to another. The object will be simulated through visible markers placed in the map at the pick up and drop off locations.

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

The turtlebot robot was prescribed for this project but, as explained later, a robot created in prior projects was used also.

### Host System

The project was delveloped under Ubuntu 16.04.7 using ROS distribution Kinetic.

## The Environment

![world](</images/environment.png>) ![turtle](</images/turtlebot.png>)

The images above show the environment that was created for the project. It is simple and uncluttered but it provides a good test of navigation and obstacle detection for our robot. We see the turtlebot robot in that environment also. This configuration of the turtlebot has the asus xtion pro 3d sensor mounted.

## Mapping

This project uses shell scripts to launch ROS nodes. The script runs each instance of roslaunch or rosrun in its own terminal. Below we have the script launching nodes to map the environment:

>#!/bin/sh

>xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &

>sleep 5

>xterm  -e  "roslaunch turtlebot_gazebo gmapping_demo.launch" &

>sleep 5

>xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

>sleep 5

>xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch" &

>sleep 5

