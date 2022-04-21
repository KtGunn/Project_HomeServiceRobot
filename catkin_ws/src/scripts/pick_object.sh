#!/bin/sh

### Created April 14 2022
### K. Gunnarsson

### Brings up Gazebo with my HSR environment.
#   We'll use this to verify that we can navigate on
#   teh map we have created.
#

export ROBOT_INITIAL_POSE="-x -4.5 -y -3.5 -z 0.0 -R 0 -P 0 -Y 0"


echo Launching turtlebot_gazebo turtlebot_world.launch
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

echo Launching turtlebot_gazebo amcl_demo.launch
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

echo Launching turtlebot_rviz_launchers view_navigation.launch
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

echo Launching pick_object pick_object_node_test
xterm  -e  "rosrun pick_objects pick_objects_node_test" &
