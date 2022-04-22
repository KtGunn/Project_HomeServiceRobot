#!/bin/sh

### Created April 14 2022
### K.G.

### Brings up Gazebo with my HSR environment.
#   We'll use this to verify that we can navigate on
#   the map we have created.
#

### 'myrobot_slammapped' is the map created gmapping with 'my_robot'
export TURTLEBOT_GAZEBO_MAP_FILE="myrobot_slammap.yaml"
export ROBOT_INITIAL_POSE="-x -4.5 -y -3.5 -z 0.0 -R 0 -P 0 -Y 0"


echo Launching turtlebot_gazebo turtlebot_world.launch
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

echo Launching turtlebot_gazebo amcl_demo.launch
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

echo Launching turtlebot_rviz_launchers view_navigation.launch
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

