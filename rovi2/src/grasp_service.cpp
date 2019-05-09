#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotiq_2f_msgs/GripperCmd.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Conf.h"
#include "std_srvs/Empty.h"
#include "rovi2/Grasp_cmd.h"
#include <string>

ros::Publisher  pub;
ros::ServiceClient wsgGripperMove;
ros::ServiceClient wsgGripperConf;
wsg_50_common::Move wsgMove;
std_srvs::Empty wsgHome;
std::string gripperID ="";
std::string node_name = ros::this_node::getName() + ": ";
bool grip = false;
bool release = false;
bool home = false;
float robo_gripDistance = 0.043;
float robo_releaseDistance = 0.1;
float robo_speed = 10.0;
float robo_force = 10.0;

float wsg_Release = 100.0;
float wsg_Grip = 53.0;
float wsg_speed = 45.0;
float wsg_force = 10.0;

bool graspObject(rovi2::Grasp_cmd::Request &req, rovi2::Grasp_cmd::Response &res)
{
  gripperID = req.ID;
  grip = req.grasp;
  release = req.release;
  home = req.homeGripper;
  ROS_INFO("gripperID: [%s]", gripperID.c_str());

  if(gripperID == "UR10B")
  {
    if(grip && !release)
      {
        robotiq_2f_msgs::GripperCmd robo_msg;
        robo_msg.emergency_release = false;
        robo_msg.emergency_release_dir =0;
        robo_msg.stop = false;
        robo_msg.position = robo_gripDistance;
        robo_msg.speed = robo_speed;
        robo_msg.force = robo_force;
        pub.publish(robo_msg);
        ROS_INFO_STREAM("Robotiq Grasping!");
        grip=false;
        return 1;
      }
    if(release && !grip)
      {
        robotiq_2f_msgs::GripperCmd robo_msg;
        robo_msg.emergency_release = false;
        robo_msg.emergency_release_dir =0;
        robo_msg.stop = false;
        robo_msg.position = robo_releaseDistance;
        robo_msg.speed = robo_speed;
        robo_msg.force = robo_force;
        pub.publish(robo_msg);
        ROS_INFO_STREAM("Robotiq Releasing!");
        release=false;
        return 1;
      }
    if(home && !grip && !release)
      {
        robotiq_2f_msgs::GripperCmd robo_msg;
        robo_msg.emergency_release = false;
        robo_msg.emergency_release_dir = 0;
        robo_msg.stop = 0;
        robo_msg.position = 0.2;
        robo_msg.speed = 10.0;
        robo_msg.force = 1.0;
        pub.publish(robo_msg);
        ROS_INFO_STREAM("Robotiq: Homing!");
        home=false;
        return 1;
      }
    else if(release && grip)
    {
      ROS_INFO("ERROR: Grasp cmd confusion! grasp and release ==True!");
      return 0;
    }
  }
    if(gripperID == "UR10A")
      {
        if(grip && !release)
          {
            wsgMove.request.width = wsg_Grip;
            wsgMove.request.speed = wsg_speed;
            ROS_INFO("wsg gripping");
            if(wsgGripperMove.call(wsgMove))
              {
                ROS_INFO_STREAM("Error: "<< (long int)wsgMove.response.error);
                return 1;
              }
            else
              {
                ROS_ERROR("Failed to call service");
                return 0;
              }
            grip=false;
          }
        if(release && !grip)
          {
            wsgMove.request.width = wsg_Release;
            wsgMove.request.speed = wsg_speed;
            ROS_INFO("wsg releasing");
            if(wsgGripperMove.call(wsgMove))
              {
                ROS_INFO_STREAM("Error: " << (long int)wsgMove.response.error);
                return 1;
              }
            else
              {
                ROS_ERROR("Failed to call service");
                return 0;
              }
            release=false;
           }
          if(home & !grip & !release)
            {
              home = false;
              ROS_INFO("wsg homing!");
              if(wsgGripperConf.call(wsgHome))
                {
                  ROS_INFO_STREAM("Done Homing" );
                  return 1;
                }
            }
          else if(release && grip)
            {
              ROS_INFO("ERROR: Grasp cmd confusion! grasp and release ==True!");
              return 0;
            }
      }
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::init(argc, argv, "listener");
  ros::init(argc, argv, "grasp_service_server");

  ros::NodeHandle nh;

  pub = nh.advertise<robotiq_2f_msgs::GripperCmd>("/gripper/cmd", 1);
  wsgGripperMove = nh.serviceClient<wsg_50_common::Move>("wsg_50_driver/move");
  wsgGripperConf = nh.serviceClient<std_srvs::Empty>("/wsg_50_driver/homing");
  ros::ServiceServer service = nh.advertiseService("grasp_cmd", graspObject);
  ROS_INFO("Ready to grasp object.");

  ros::spin();

  return 0;
}
