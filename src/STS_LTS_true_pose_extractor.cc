#include "STS_LTS_pose_plotter.h"

#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
  std::cout << "*** true pose extractor starts running ***" << std::endl;
  ros::init(argc, argv, "pose_plotter");

  // Subscribe to nodes.
  ros::NodeHandle n;
  //ros::Subscriber sub_sts = n.subscribe("/fw_pose/sts", 1000, &PosePlotter::poseCallbackSTS, &plotter);
  //ros::Subscriber sub_lts = n.subscribe("/fw_pose/lts", 1000, &PosePlotter::poseCallbackLTS, &plotter);

  ros::spin();

  std::cout << "*** true pose extractor finished ***" << std::endl;
  return 0;
}
