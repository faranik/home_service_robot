#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach the pickup zone.
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base moved to pickup zone.");
    ros::Duration(5.0).sleep();
  
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -4.0;
    goal.target_pose.pose.position.y = -2.0;
    goal.target_pose.pose.orientation.w = 4.14;

    // Send the goal position and orientation for the robot to reach the dropdown zone.
    ROS_INFO("Sending dropdown goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved to dropdown zone.");
    else
      ROS_INFO("The base failed to move to dropdown zone for some reason");
  }
  else
    ROS_INFO("The base failed to move to pickup zone for some reason");

  ros::Duration(5.0).sleep();
  return 0;
}
