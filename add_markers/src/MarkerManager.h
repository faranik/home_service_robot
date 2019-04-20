#include <ros/ros.h>
#include <string>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class MarkerManager
{
public:
  MarkerManager();
  ~MarkerManager(){}

  void addMarker(const geometry_msgs::Pose& pose, 
                 const int marker_id,
                 const uint8_t shape);
  
  void removeMarker(const int marker_id);
  
  void setPublisher(ros::NodeHandle& handle);

private:
  void publish();
  
  visualization_msgs::Marker marker;
  std::shared_ptr<ros::Publisher> marker_pub;
};
