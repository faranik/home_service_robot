#include <ros/ros.h>
#include <mover/Move.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish() :
    ac("move_base", true)
  {
    // Define a move service with a handle_move_request callback function
    srv = n.advertiseService("/robot_mover", &SubscribeAndPublish::handleMoveRequest, this);
  }

  // This callback function executes whenever a move service is requested
  bool handleMoveRequest(mover::Move::Request& req, mover::Move::Response& res)
  {
    // Wait for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    float x = (float)req.x;
    float y = (float)req.y;
    float theta = (float)req.theta;
    ROS_INFO("Move Robot To Position Request received - x:%1.2f, y:%1.2f, theta:%1.2f", x, y, theta);
  
    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach the pickup zone.
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = theta;

    // Send the goal position and orientation for the robot to reach
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("The robot reached its destination successfully.");
      res.status = 0;
    }
    else
    {
      ROS_INFO("The robot failed to reach its destination.");
      res.status = 1;
    }

    ROS_INFO_STREAM(res.status);

    return true;
  }


private:
  ros::NodeHandle n; 
  ros::ServiceServer srv;
  
  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "mover");

  // Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  // Wait in peace.
  ros::spin();

  return 0;
}

