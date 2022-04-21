#!/bin/sh

### Created April 14 2022
### K. Gunnarsson

### Brings up Gazebo with my HSR environment and also the gmapping
#   package. Adidtionally the teleop node (to drive the robot)
#   the Rviz (to vizulaize the map) are brought up
#   We'll use this to verify that we can create a map with
#   this robot and associated packages.
#

echo Launching turtlebot_gazebo turtlebot_world.launch
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

echo Launching turtlebot_gazebo gmapping_demo.launch
xterm  -e  "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

echo Launching turtlebot_rviz_launchers view_navigation.launch
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

echo Launching turtlebot_teleop keyboard_teleop.launch
xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch" &

echo All launches are completed.
