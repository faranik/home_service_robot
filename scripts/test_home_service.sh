#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/catkin_ws/src/home_service_robot/map/playground.world " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/robond/catkin_ws/src/home_service_robot/map/playground.yaml" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " rosrun mover mover_node " &
sleep 5
xterm  -e  " rosrun obj_manipulator obj_manipulator_node " &
sleep 5
xterm  -e  " rosrun plan_executor plan_executor_node " &
