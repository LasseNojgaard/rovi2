#include "ros/ros.h"
#include "pose_ros/Object_pose.h"
#include "rovi2/Grasp_cmd.h"

//#include "rovi2/src/SBLcollision.cpp"
#include <string>
ros::Subscriber visionSub;
ros::ServiceClient grasp_client;
//ros::ServiceClient SBL_client;

float objectLocations[2][6];  // x, y, z, roll, pitch, yaw;
int foundObjects = 0;
bool objectRecived[2]={false, false};
std::string rosInfo;

void visionInfo(const pose_ros::Object_pose::ConstPtr& pose)
{
  ROS_INFO("In callback");
  if(objectRecived[pose->ID] == false)
  {
    foundObjects++;
    objectLocations[pose->ID][0] = pose->x;
    objectLocations[pose->ID][1] = pose->y;
    objectLocations[pose->ID][2] = pose->z;
    objectLocations[pose->ID][3] = pose->roll;
    objectLocations[pose->ID][4] = pose->pitch;
    objectLocations[pose->ID][5] = pose->yaw;

    if(pose->ID ==0 ) {
      rosInfo = "PoseA: ";
      for (size_t i = 0; i < 5; i++) {
          rosInfo += std::to_string(objectLocations[0][i]);
            rosInfo+= " , ";
      }
    }
    else {
        rosInfo= "PoseB: ";
        for (size_t j = 0; j < 5; j++) {
            rosInfo += std::to_string(objectLocations[1][j]);
            rosInfo+= " , ";
        }

    }
      ROS_INFO("[%s]", rosInfo.c_str());

      objectRecived[pose->ID]=true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::init(argc, argv, "listener");
  ros::init(argc, argv, "grasp_service_server");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ROS_INFO("Started Code");
  visionSub     = nh.subscribe("object_pose", 3, visionInfo);
  grasp_client  = nh.serviceClient<rovi2::Grasp_cmd>("grasp_cmd");
//  SBL_client    = nh.serviceClient<SBLcollision::SBL_cmd>("SBL_cmd");

  while (ros::ok()){
    if(foundObjects > 1){
      /*
            SBL_client.request.tAx = objectLocations[0][0];
            SBL_client.request.tAy = objectLocations[0][1];
            SBL_client.request.tAz = objectLocations[0][2];
            SBL_client.request.rAx = objectLocations[0][3];
            SBL_client.request.rAy = objectLocations[0][4];
            SBL_client.request.rAz = objectLocations[0][5];
            SBL_client.request.tBx = objectLocations[1][0];
            SBL_client.request.tBy = objectLocations[1][1];
            SBL_client.request.tBz = objectLocations[1][2];
            SBL_client.request.rBx = objectLocations[1][3];
            SBL_client.request.rBy = objectLocations[1][4];
            SBL_client.request.rBz = objectLocations[1][5];
            SBL_client.request.goal = false;
            foundObjects =0;

            rosInfo = "PoseA: "
              for (size_t i = 0; i < 5; i++) {
                  rosInfo += objectLocations[0][j];
              }
              rosInfo+= "PoseB: ")
                for (size_t j = 0; j < 5; j++) {
                    rosInfo += objectLocations[1][j];
                }
              ROS_INFO(rosInfo);
          }

        if(SBLcollision.call(SBL_client))
          {
            ROS_INFO("Moved succeeded");
          }
        else
          {
            ROS_ERROR("Failed to call SBL service");
            return 0;
          }
        grasp_client.request.ID = "UR10A";
        grasp_client.request.grasp = true;
        grasp_client.request.release = false;
        grasp_client.request.homeGripper = false;
        grasp_client.request.speed = 25.0;
        grasp_client.request.force = 5.0;
        grasp_service.call(grasp_client);

        grasp_client.request.ID = "UR10B";
        grasp_client.request.grasp = true;
        grasp_client.request.release = false;
        grasp_client.request.homeGripper = false;
        grasp_client.request.speed = 25.0;
        grasp_client.request.force = 5.0;
        grasp_service.call(grasp_client);

        SBL_client.request.tAx = objectLocations[0][0];
        SBL_client.request.tAy = objectLocations[0][1];
        SBL_client.request.tAz = objectLocations[0][2];
        SBL_client.request.rAx = objectLocations[0][3];
        SBL_client.request.rAy = objectLocations[0][4]+0.15;
        SBL_client.request.rAz = objectLocations[0][5];
        SBL_client.request.tBx = objectLocations[1][0];
        SBL_client.request.tBy = objectLocations[1][1];
        SBL_client.request.tBz = objectLocations[1][2];
        SBL_client.request.rBx = objectLocations[1][3];
        SBL_client.request.rBy = objectLocations[1][4]+0.15;
        SBL_client.request.rBz = objectLocations[1][5];
        SBL_client.request.goal = false;
        if(SBLcollision.call(SBL_client))
          {
            ROS_INFO("Moved succeeded");
          }
        else
          {
            ROS_ERROR("Failed to call SBL service");
            return 0;
          }

        SBL_client.request.tAx = 0;
        SBL_client.request.tAy = 0;
        SBL_client.request.tAz = 0;
        SBL_client.request.rAx = 0;
        SBL_client.request.rAy = 0;
        SBL_client.request.rAz = 0;
        SBL_client.request.tBx = 0;
        SBL_client.request.tBy = 0;
        SBL_client.request.tBz = 0;
        SBL_client.request.rBx = 0;
        SBL_client.request.rBy = 0;
        SBL_client.request.rBz = 0;
        SBL_client.request.goal = true;
        if(SBLcollision.call(SBL_client))
          {
            ROS_INFO("Moved succeeded");
          }
        else
          {
            ROS_ERROR("Failed to call SBL service");
            return 0;
          }

        grasp_client.request.ID = "UR10A";
        grasp_client.request.grasp = false;
        grasp_client.request.release = true;
        grasp_client.request.homeGripper = false;
        grasp_client.request.speed = 25.0;
        grasp_client.request.force = 5.0;
        grasp_service.call(grasp_client);

        grasp_client.request.ID = "UR10B";
        grasp_client.request.grasp = false;
        grasp_client.request.release = true;
        grasp_client.request.homeGripper = false;
        grasp_client.request.speed = 25.0;
        grasp_client.request.force = 5.0;
        grasp_service.call(grasp_client);
        */
        foundObjects = 0;
        objectRecived[0]=false;
        objectRecived[1]=false;
  }
  ros::spinOnce();
}
  return 0;
}
