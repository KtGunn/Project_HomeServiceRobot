#!/bin/sh

### Created March 26 2033
### K.G.

echo Launching Gazebo
xterm  -e  " gazebo " &
sleep 5

echo Starting Rosmaster
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5

echo Starting Rviz
xterm  -e  " rosrun rviz rviz" 
