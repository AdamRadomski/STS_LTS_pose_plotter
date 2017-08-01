#include "STS_LTS_pose_plotter.h"

#include <geometry_msgs/PoseStamped.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void poseCallbackSTS(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("STS heard a pose.");

}

void poseCallbackLTS(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("LTS heard a pose.");

}

int main(int argc, char **argv)
{
  std::cout << "*** pose plotter starts running ***" << std::endl;


  ros::init(argc, argv, "pose_plotter");

  // Subscribe to nodes.
  ros::NodeHandle n;
  ros::Subscriber sub_sts = n.subscribe("/fw_pose/sts", 1000, poseCallbackSTS);
  ros::Subscriber sub_lts = n.subscribe("/fw_pose/lts", 1000, poseCallbackLTS);

  ros::spin();

  std::cout << "*** pose plotter finished ***" << std::endl;
  return 0;
}
