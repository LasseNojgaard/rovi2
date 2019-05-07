#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotiq_2f_msgs/GripperCmd.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Conf.h"
#include "std_srvs/Empty.h"
#include "rovi2/Grasp_cmd.h"
#include <string>

std::string gripperID ="";
std::string node_name = ros::this_node::getName() + ": ";
bool publish = false;
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

void graspCallback(const rovi2::Grasp_cmd::ConstPtr& msg)
{
  gripperID = msg->ID;
  publish = true;
  grip = msg->grasp;
  release = msg->release;
  home = msg->homeGripper;

  if(gripperID == "robotiq")
    {
      robo_speed = msg->speed;
      robo_force = msg->force;
    }
  if(gripperID == "wsg")
    {
      wsg_speed = msg->speed;
      wsg_force = msg->force;
    }
    ROS_INFO("gripperID: [%s]", gripperID.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("Grasp", 1, graspCallback);
  ros::Publisher  pub = nh.advertise<robotiq_2f_msgs::GripperCmd>("/gripper/cmd", 1);
  ros::ServiceClient wsgGripperMove = nh.serviceClient<wsg_50_common::Move>("wsg_50_driver/move");
  ros::ServiceClient wsgGripperConf = nh.serviceClient<std_srvs::Empty>("/wsg_50_driver/homing");

  wsg_50_common::Move wsgMove;
  std_srvs::Empty wsgHome;

  ros::Rate loop_rate(10);


  while(ros::ok())
  {
    if(publish)
    {
      publish=false;
      if(gripperID == "robotiq")
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
          }
        else if(release && grip)
        {
          ROS_INFO("ERROR: Grasp cmd confusion! grasp and release ==True!");
        }
      }
        if(gripperID == "wsg")
          {
            if(grip && !release)
              {
                wsgMove.request.width = wsg_Grip;
                wsgMove.request.speed = wsg_speed;
                ROS_INFO("wsg gripping");
                if(wsgGripperMove.call(wsgMove))
                  {
                    ROS_INFO_STREAM("Error: "<< (long int)wsgMove.response.error);
                  }
                else
                  {
                    ROS_ERROR("Failed to call service");
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
                  }
                else
                  {
                    ROS_ERROR("Failed to call service");
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
                    }
                }
              else if(release && grip)
                {
                  ROS_INFO("ERROR: Grasp cmd confusion! grasp and release ==True!");
                }
          }
      }
  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}
