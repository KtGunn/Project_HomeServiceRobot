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

![turtleslam](</images/turtleslam.png>)
 
The figure above shows the start of mapping using the turtle bot (see *.../src/scripts/test_slam.sh*). The figure show the limited range the 3d sensor. The result was that mapping with this sensor failed. After painstakingly slow and drawn-out mapping operation this was the result (see below).

![tmap](</images/turtlemap.png>)

A robot created in a prior project was used to perform the mapping (see *.../src/srcipt/myrobot_slam.sh*). The gif below shows a snippet of the mapping operation, which successfully created a map.

![myrmap](</images/myrobotmapping.gif>)

Although much better success was had with my own robot, thanks to the hokuyo lidar mounted on it, slam mapping is still a touchy operation. One must move slowly and seek to attain closures to build up a reliable and correct map. If a robot starts jumping around much during mapping, closure may not be attained and one must start over.

![turtleslam](</images/myrobotslammap.png>)

## Navigation

Once a map is available the move_base package for navigating can be tested. Two scripts were created for this purpose:

- src/scripts/test_navigation.sh
- src/scripts/test_navigation_gmapped.sh

Before the SLAM difficulties with turtlebot had been sorted out, a map was created using the pgm_map_creator. This software creates the map directly from Gazebo environment by projecting walls (and other objects) down to the 2d plane. A very clean map is created. This map is used in the first script above. The second script uses the SLAM created map. The animation below demonstrates navigation on the SLAM created map.

![turtnav](</images/turtlenavigating.gif>)

## Home Service Operation

The work presented above builds up to the main task, the simulation of a home service robot. Two packages were created:

- pick_object
- add_markers

The *pick_object* package moves the robot (turtlebot) between the pick up and drop off locations. It communicates with the move_base package. The *add_markers* package simulates the object by placing markers on the map.

The operation starts when a marker appears at the pick up location. The robot then moves to the pick up location and the marker disappears. A short pause imitates the actual picking up of the object. Now the tug moves to the drop of location. Once at the drop off location the marker reappeaers indicating the object has been delivered. The process is repeated indefinitely.

### Coordination

The two packages or nodes coordinate their operations using **semaphores** i.e. parameters set on the rosmaster. In brief,

> At start up robot (*pick_object*) will post **robot_ready=true** to master;

> At start add_markers will post **target_set=false** and **robot_atTarget=false**;

> A cycle now starts:

> Marker sees robot_ready==true and places the marker in the pick up location and

> >sets the pickup location, **pikk_x** and **pikk_y**,

> >sets **target_set=true**;

> >drops **robot_atTarget=false**;

> Robot sees target is set, reads the location, and

> >sets **robot_ready=false**

> >sets **target_set=false**

> and moves to the location;

> Robot arrives at target location and

> > sets **robot_atTarget=true** then,

> > sets **robot_ready=true**;

> Marker sees **robot_atTarget** and **robot_ready** are true, deletes the marker and pauses;

> Marker sees **robot_ready==true** and sets a new location as above;

> Robot see **target_set==true** again and signals and moves as above;

> Marker sees **robot_atTarge==true** waits for **robot_ready==true** again and places marker;

> Marker pauses and announces object has been delivered.

The process repeats endlessly.


