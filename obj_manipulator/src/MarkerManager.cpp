#include <string>
#include <math.h>
#include "MarkerManager.h"


MarkerManager::MarkerManager()
{
  // Set the frame ID and the timestamp. See the TF tutorials for information on these topics
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the marker namespace and id.This serves to create a unique ID.
  // Any marker sent with the same namespace and id will overwrite the old one.
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
}


void MarkerManager::addMarker(const geometry_msgs::Pose& pose,
                              const int marker_id,
                              const uint8_t shape)
{
  if(!marker_pub)
  {
    ROS_WARN("Please set the publisher.");
    return;
  } 

  // Set the shape and ID.
  marker.type = shape;
  marker.id = marker_id;

  // Set the marker action.  Options are ADD, MODIFY, DELETE, DELETEALL
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;
  marker.pose.orientation.x = pose.orientation.x;
  marker.pose.orientation.y = pose.orientation.y;
  marker.pose.orientation.z = pose.orientation.z;
  marker.pose.orientation.w = pose.orientation.w;

  marker.lifetime = ros::Duration();

  publish();
}


void MarkerManager::removeMarker(const int marker_id)
{
  if(!marker_pub)
  {
    ROS_WARN("Please set the publisher.");
    return;
  } 

  // Set the ID of marker to delete.
  marker.id = marker_id;

  // Set the marker action.  Options are ADD, MODIFY, DELETE, DELETEALL
  marker.action = visualization_msgs::Marker::DELETE;

  publish();
}

void MarkerManager::setPublisher(ros::NodeHandle& n)
{
  // Initialize the publisher.
  this->marker_pub = std::make_shared<ros::Publisher>(n.advertise<visualization_msgs::Marker>("visualization_marker", 1));
}

void MarkerManager::publish()
{
  // Publish the marker
  while (marker_pub->getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
  }
  marker_pub->publish(marker);
}
