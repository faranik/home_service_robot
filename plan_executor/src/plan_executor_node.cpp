#include <ros/ros.h>
#include <mover/Move.h>
#include <obj_manipulator/ObjManipulation.h>

class SubscribeAndPublish
{
public:
  
  SubscribeAndPublish()
  {
    // Register as client of mover service.
    moverClient = n.serviceClient<mover::Move>("/robot_mover");

    // Register as client of object manipulator service.
    objManipulatorClient = n.serviceClient<obj_manipulator::ObjManipulation>("/object_manipulator");
  }

  void run()
  {
    MoverRequest requestsToMove[5] = {{4.0, -2.0, 1.0},
                      {4.0, -1.0, 1.0},
                      {4.0, 0.0, 1.0},
                      {4.0, 1.0, 1.0},
                      {4.0, 2.0, 1.0}};

    ObjManipulationRequest requestsToManipulate[5] = {{0, 0, 4.0, -2.0},
                            {1, 0, 4.0, -1.0},
                            {2, 0, 4.0, 0.0},
                            {3, 0, 4.0, 1.0},
                            {4, 0, 4.0, 2.0}};
    ROS_INFO("Place the objects");

    for(int i = 0; i < 5; ++i)
    {
      obj_manipulator::ObjManipulation manipulationSrv;
      manipulationSrv.request.id = requestsToManipulate[i].id;
      manipulationSrv.request.command = requestsToManipulate[i].command;
      manipulationSrv.request.x = requestsToManipulate[i].x;
      manipulationSrv.request.y = requestsToManipulate[i].y;

      if(!objManipulatorClient.call(manipulationSrv))
      {
        ROS_WARN("Object manipulation service has some problem to place object into environment.");
      }
    }

    ROS_INFO("Move robot");

    float delta = 0.0;
    for(int i = 0; i < 5; ++i)
    {
      mover::Move moverSrv;
      moverSrv.request.x = requestsToMove[i].x;
      moverSrv.request.y = requestsToMove[i].y;
      moverSrv.request.theta = requestsToMove[i].theta;
      
      if(!moverClient.call(moverSrv))
      {
        continue;
      }

      obj_manipulator::ObjManipulation manipulationSrv;
      manipulationSrv.request.id = requestsToManipulate[i].id;
      manipulationSrv.request.command = 1;
      manipulationSrv.request.x = requestsToManipulate[i].x;
      manipulationSrv.request.y = requestsToManipulate[i].y;

      if(!objManipulatorClient.call(manipulationSrv))
      {
        continue;
      }

      moverSrv.request.x = -4.0; 
      moverSrv.request.y = -2.0 - delta;
      moverSrv.request.theta = requestsToMove[i].theta;
      
      if(!moverClient.call(moverSrv))
      {
        continue;
      }
      
      manipulationSrv.request.id = requestsToManipulate[i].id;
      manipulationSrv.request.command = requestsToManipulate[i].command;
      manipulationSrv.request.x = -4.0;
      manipulationSrv.request.y = -2.0 - delta;

      if(!objManipulatorClient.call(manipulationSrv))
      {
        continue;
      }

      delta += 0.3;
    }
  }

private:
  ros::NodeHandle n; 
  ros::ServiceClient moverClient;
  ros::ServiceClient objManipulatorClient;

  struct MoverRequest
  {
    float x;
    float y;
    float theta;
  };

  struct ObjManipulationRequest
  {
    int id;
    int command;
    float x;
    float y;
  };

};


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "plan_executor");

  if(ros::ok())
  {
    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;
    SAPObject.run();

    ros::spin();
  }

  return 0;
}

