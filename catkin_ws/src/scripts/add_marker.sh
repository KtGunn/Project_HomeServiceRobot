#!/bin/sh

### Created April 14 2022
### K.G.

### Brings up Gazebo with my HSR environment.
#   We'll use this to verify that we can place
#   markers in the map
#

export ROBOT_INITIAL_POSE="-x -4.5 -y -3.5 -z 0.0 -R 0 -P 0 -Y 0"
export TURTLEBOT_GAZEBO_MAP_FILE="myrobot_slammap.yaml"


echo Launching turtlebot_gazebo turtlebot_world.launch
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

echo Launching turtlebot_gazebo amcl_demo.launch
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

echo Launching turtlebot_rviz_launchers view_navigation.launch
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

echo Running add_marker add_marker_node_test
xterm  -e  "rosrun add_markers add_markers_node_test" &

