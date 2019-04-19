#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

const float PICKUP_X = 2.0;
const float PICKUP_Y = 1.0;
const float DROPOFF_X = -4.0;
const float DROPOFF_Y = -2.0;
const float PICKUP_Z = 0.1;
const float DROPOFF_Z = 0.1;

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

bool is_picked = false;
bool is_droped = false;

void watch_robot_position(const nav_msgs::Odometry odom)
{
  float DELTA = 0.01;

  if(abs(odom.pose.pose.position.x - PICKUP_X) < DELTA &&
     abs(odom.pose.pose.position.y - PICKUP_Y) < DELTA &&
     !is_picked)
  {
    // Remove the marker.
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    is_picked = true;
  }

  if(abs(odom.pose.pose.position.x - DROPOFF_X) < DELTA &&
     abs(odom.pose.pose.position.y - DROPOFF_Y) < DELTA &&
     !is_droped)
  {
    // Add the marker at dropoff zone location
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = DROPOFF_X;
    marker.pose.position.y = DROPOFF_Y;
    marker.pose.position.z = DROPOFF_Z;

    marker_pub.publish(marker);
    is_droped = true;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  if (ros::ok())
  { 
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = PICKUP_X;
    marker.pose.position.y = PICKUP_Y;
    marker.pose.position.z = PICKUP_Z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(3);
    }
    marker_pub.publish(marker);

  }
    
  // Subscribe to /odom topic to read the position of turtlebot.
  ros::Subscriber sub = n.subscribe("odom", 10, watch_robot_position);


  // Handle ROS communication events
  ros::spin();

  return 0;
}
