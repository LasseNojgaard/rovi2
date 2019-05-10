#include "ros/ros.h"
#include "pose_ros/Object_pose.h"
#include "rovi2/Grasp_cmd.h"

#include <rw/math/Q.hpp>
#include <string>

ros::NodeHandle nh;
ros::Rate loop_rate(10);
ros::Subscriber visionSub;
ros::ServiceClient grasp_client;
ros::ServiceClient SBL_client;

float objectLocations[3][6];  // x, y, z, roll, pitch, yaw;
bool objectRecived[3]={false,false,false};


void visionInfo(pose_ros::Object_pose::Ptr pose)
{
  if(objectRecived[pose->ID] == fasle)
  {
    objectLocations[pose->ID][0] = pose->x;
    objectLocations[pose->ID][1] = pose->y;
    objectLocations[pose->ID][2] = pose->z;
    objectLocations[pose->ID][3] = pose->roll;
    objectLocations[pose->ID][4] = pose->pitch;
    objectLocations[pose->ID][5] = pose->yaw;
    objectRecived[pose->ID] =true;
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::init(argc, argv, "listener");
  ros::init(argc, argv, "grasp_service_server");

  visionSub = nh.subscribe("object_pose", 3, visionInfo);
  grasp_client = nh.serviceClient<grasp_service::Grasp_cmd>("grasp_cmd");
  SBL_client = nh.serviceClient</*indsæt service*/>(/*service command*/);

  while (ros::ok())
  {
    // get object pose

    //calc inverse kinematic

    //find SBL path

    //grasp

    //Lift up

    //calc inverse kinematic

    //find SBL path

    //release

    ros::spinOnce();
  }

  return 0;
}
