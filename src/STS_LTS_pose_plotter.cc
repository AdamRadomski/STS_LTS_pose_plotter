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

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t timestamp = rosTimeToNanoseconds(msg.header.stamp);
    sts_timestamp_to_pose_map.insert(std::make_pair(timestamp, msg));

    // Check if none of timestamps was duplicated.
    assert(sts_msg_count_ == sts_timestamp_to_pose_map.size());
  }

  // Calculate error between estimators.
  calcErrorBetweenEstimators();
}

void PosePlotter::poseCallbackLTS(const geometry_msgs::PoseStamped& msg)
{
  ++lts_msg_count_;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t timestamp = rosTimeToNanoseconds(msg.header.stamp);
    lts_timestamp_to_pose_map.insert(std::make_pair(timestamp, msg));

    // Check if none of timestamps was duplicated.
    assert(lts_msg_count_ == lts_timestamp_to_pose_map.size());
  }


  // Calculate error between estimators.
  calcErrorBetweenEstimators();
}

void PosePlotter::calcErrorBetweenEstimators(){

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
