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

The project uses shell scripts to launch ROS nodes. The script runs each instance of roslaunch or rosrun in its own terminal. The script for mapping executes four roslaunch commands:

  >roslaunch turtlebot_gazebo turtlebot_world.launch\
  >roslaunch turtlebot_gazebo gmapping_demo.launch\
  >roslaunch turtlebot_rviz_launchers view_navigation.launch\
  >roslaunch turtlebot_teleop keyboard_teleop.launch\

The first launch brings up Gazebo showing the environment and turtlebot. The second launch runs the SLAM mapping node which has no direct gui interface only report to the console. The third launch brings up rviz in which we observe the progress of the mapping operation as the tug moves and the mapping node builds up the map. Finally the fourth node bring up the keyboard tele operation where through single key commands the robot is moved forward backward and rotated clock- and counter-clockwise.

![turtle](</images/turtleslam.png>)
 
The figure above shows the start of mapping using the turtle bot. The figure show the limited range the 3d sensor. The result was that mapping with this sensor failed. After painstakingly slow and drawn-out mapping operation this was the result (see below).

![turtle](</images/turtlemap.png>)



