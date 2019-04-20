#include "MarkerManager.h"


class Subscriber
{
public:
  Subscriber()
  {
    // Subscribe to odom topic and set the callback.
    sub_ = n_.subscribe("odom", 10, &Subscriber::callback, this);
    
    // Set the marker publisher.
    manager.setPublisher(n_);
    
    geometry_msgs::Pose pickupPose;
    pickupPose.position.x = PICKUP_X;
    pickupPose.position.y = PICKUP_Y;
    pickupPose.position.z = PICKUP_Z;
    pickupPose.orientation.x = 0.0;
    pickupPose.orientation.y = 0.0;
    pickupPose.orientation.z = 0.0;
    pickupPose.orientation.w = 1.0;

    manager.addMarker(pickupPose, 0, visualization_msgs::Marker::CUBE);
  }

  void callback(const nav_msgs::Odometry& odom)
  {
    if(abs(odom.pose.pose.position.x - PICKUP_X) < DELTA &&
       abs(odom.pose.pose.position.y - PICKUP_Y) < DELTA &&
       !is_picked)
    {  
      // Remove the marker.
      manager.removeMarker(0);
      is_picked = true;
    }

    if(abs(odom.pose.pose.position.x - DROPOFF_X) < DELTA &&
       abs(odom.pose.pose.position.y - DROPOFF_Y) < DELTA &&
       !is_droped)
    {
      geometry_msgs::Pose dropoffPose;
      dropoffPose.position.x = DROPOFF_X;
      dropoffPose.position.y = DROPOFF_Y;
      dropoffPose.position.z = DROPOFF_Z;
      dropoffPose.orientation.x = 0.0;
      dropoffPose.orientation.y = 0.0;
      dropoffPose.orientation.z = 0.0;
      dropoffPose.orientation.w = 1.0;

      manager.addMarker(dropoffPose, 0, visualization_msgs::Marker::CUBE);
      is_droped = true;
    }
  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;

  const float DELTA = 0.01;

  const float PICKUP_X = 2.0;
  const float PICKUP_Y = 1.0;
  const float PICKUP_Z = 0.1;

  const float DROPOFF_X = -4.0;
  const float DROPOFF_Y = -2.0;
  const float DROPOFF_Z = 0.1;

  bool is_droped = false;
  bool is_picked = false;

  MarkerManager manager;

};//End of class SubscribeAndPublish


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  if (ros::ok())
  { 
    //Create an object of class SubscribeAndPublish that will take care of everything
    Subscriber subscriber;

    ros::spin();
  }

  return 0;
}
