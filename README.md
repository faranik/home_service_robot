# home_service_robot
This project builds a Home Service Robot in ROS. It includes robot localization, mapping (SLAM) and path planning.

This project is part of the Robotics Software Engineering scholar course offerd at Udacity. It explores concepts like robot localization, environment mapping and path planning, all in the context of ROS Kinetic, robotic operating system.

## Get started
For convinience, install **xterm** to be able to start the entire solution by executing the **launch.sh** file.
```
sudo apt install xterm
```

Four official ROS packages should be cloned to the ```src``` folder of your ```catkin workspace```. Be sure to clone the full GitHub directory and not just the package itself.

1. [gmapping](http://wiki.ros.org/gmapping): With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
2. [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop): With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
3. [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers): With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
4. [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo): With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

Or you can just run the following command into a terminal:
```
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_interactions.git
git clone https://github.com/turtlebot/turtlebot_simulator.git
```

## Troubleshooting
If you run the test_navigation.sh file and the turtlebot robot is pirouetting after you set a ```2D Nav Goal``` then you probably have some inconsistencies with parameters of move base between local and global planner. Try to edit ```dwa_local_planner_params.yaml``` file from ```turtlebot_navigation``` package. In my case it is at this location ```/opt/ros/kinetic/share/turtlebot_navigation/param/dwa_local_planner_params.yaml```. The parameters that worked for me are:
```
  max_trans_vel: 0.3 # choose slightly less than the base's capability
  min_trans_vel: 0.1  # this is the min trans velocity when there is negligible rotational velocity
  trans_stopped_vel: 0.1

  # Warning!
  #   do not set min_trans_vel to 0.0 otherwise dwa will always think translational velocities
  #   are non-negligible and small in place rotational velocities will be created.

  max_rot_vel: 0.5  # choose slightly less than the base's capability
  min_rot_vel: 0.4  # this is the min angular velocity when there is negligible translational velocity
  rot_stopped_vel: 0.3
```
