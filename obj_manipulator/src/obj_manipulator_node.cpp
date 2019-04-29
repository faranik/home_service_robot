#include "MarkerManager.h"
#include <obj_manipulator/ObjManipulation.h>

class Subscriber
{
public:
  Subscriber()
  {
    // Define an object manipulator service.
    srv = n.advertiseService("/object_manipulator", &Subscriber::handleObjManipulationRequest, this);

    // Set the marker publisher.
    manager.setPublisher(n);
  }

  // This callback function executes whenever a manipulate object service is requested.
  bool handleObjManipulationRequest(obj_manipulator::ObjManipulation::Request& req, 
                                    obj_manipulator::ObjManipulation::Response& res)
  {
    res.status = OK;

    geometry_msgs::Pose pose;
    pose.position.x = (float)req.x;
    pose.position.y = (float)req.y;

    // In the context of this small project these values are constant.
    pose.position.z = 0.1;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    if((int)req.command == ADD)
    {
      ROS_INFO("Place a cube at position - x:%1.2f, y:%1.2f", pose.position.x, pose.position.y);

      // For the purpose of this project we always work with cubes.
      manager.addMarker(pose, (int)req.id, visualization_msgs::Marker::CUBE);
    }
    else if((int)req.command == REMOVE)
    {
      ROS_INFO("Remove the cube with id:%d", (int)req.id);
      manager.removeMarker((int)req.id);
    }
    else
    {
      ROS_WARN("Object manipulator node received an unknown command :%d", (int)req.command);
      res.status = FAIL;
    }

    ROS_INFO_STREAM(res.status);
    return true;
  }

private:
  ros::NodeHandle n; 
  ros::ServiceServer srv;

  MarkerManager manager;

  const int ADD = 0;
  const int REMOVE = 1;
  const int OK = 0;
  const int FAIL = 1;

};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "obj_manipulator");

  if (ros::ok())
  { 
    //Create an object of class Subscribe that will take care of everything
    Subscriber subscriber;

    ros::spin();
  }

  return 0;
}
