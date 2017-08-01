#include "STS_LTS_pose_plotter.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

PosePlotter::PosePlotter() {
  sts_msg_count_ = 0u;
  lts_msg_count_ = 0u;
}

void PosePlotter::poseCallbackSTS(const geometry_msgs::PoseStamped& msg)
{
  ++sts_msg_count_;

  ROS_INFO("STS heard a pose. [%d]", sts_msg_count_);

}

void PosePlotter::poseCallbackLTS(const geometry_msgs::PoseStamped& msg)
{
  ++lts_msg_count_;

  ROS_INFO("LTS heard a pose. [%d]", lts_msg_count_);

}

int main(int argc, char **argv)
{
  std::cout << "*** pose plotter starts running ***" << std::endl;
  ros::init(argc, argv, "pose_plotter");

  // Subscribe to nodes.
  PosePlotter plotter;
  ros::NodeHandle n;
  ros::Subscriber sub_sts = n.subscribe("/fw_pose/sts", 1000, &PosePlotter::poseCallbackSTS, &plotter);
  ros::Subscriber sub_lts = n.subscribe("/fw_pose/lts", 1000, &PosePlotter::poseCallbackLTS, &plotter);

  ros::spin();

  std::cout << "*** pose plotter finished ***" << std::endl;
  return 0;
}
