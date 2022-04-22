#!/bin/sh

### Created April 18 2022
### K.G.

### Brings up all packages to implement the 
#     Home Service Robot Project
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

echo Launching pick_object pick_object_node
xterm  -e  "rosrun pick_objects pick_objects_node" &

echo Launching add_markers add_markers_node
xterm  -e  "rosrun add_markers add_markers_node" &


