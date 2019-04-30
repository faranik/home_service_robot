#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " rosrun mover mover_node " &
sleep 5
xterm  -e  " rosrun obj_manipulator obj_manipulator_node " &
sleep 5
xterm  -e  " rosrun plan_executor plan_executor_node " &
