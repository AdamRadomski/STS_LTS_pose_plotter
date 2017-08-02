#include "STS_LTS_pose_plotter.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "common.h"

PosePlotter::PosePlotter() {
  sts_msg_count_ = 0u;
  lts_msg_count_ = 0u;
  last_found_timestamp_idx_ = 0u;
}

void PosePlotter::poseCallbackSTS(const geometry_msgs::PoseStamped& msg)
{
  ++sts_msg_count_;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t timestamp = rosTimeToNanoseconds(msg.header.stamp);
    sts_timestamp_to_pose_map_.insert(std::make_pair(timestamp, msg));
    available_timestamps_.insert(timestamp);

    // Check if none of timestamps was duplicated.
    assert(sts_msg_count_ == sts_timestamp_to_pose_map_.size());
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
    lts_timestamp_to_pose_map_.insert(std::make_pair(timestamp, msg));
    available_timestamps_.insert(timestamp);

    // Check if none of timestamps was duplicated.
    assert(lts_msg_count_ == lts_timestamp_to_pose_map_.size());
  }


  // Calculate error between estimators.
  calcErrorBetweenEstimators();
}

void PosePlotter::calcErrorBetweenEstimators(){
  ROS_INFO("set size %u", available_timestamps_.size());
  assert(last_found_timestamp_idx_ < available_timestamps_.size());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Try to find pose messages for the next available timestamp.
    std::set<int64_t>::iterator it_ts = available_timestamps_.begin();
    // Initialize iterator to the last checked position.
    for (size_t i = 0u ; i < last_found_timestamp_idx_ ; ++i){
      ++it_ts;
    }
    for (  ; it_ts != available_timestamps_.end() ; ++it_ts){
      const int64_t& timestamp = *it_ts;
      //ROS_INFO("Timestamp ");
      std::cout << " " << timestamp << std::endl;

      // Try to find this timestamp in LTS because LTS is slightly delayed.
      TimestampToPoseMap::iterator it_lts = lts_timestamp_to_pose_map_.find(timestamp);
      if (it_lts == lts_timestamp_to_pose_map_.end()){
        // Try again later.
        break;
      }

      TimestampToPoseMap::iterator it_sts = sts_timestamp_to_pose_map_.find(timestamp);
      if (it_sts == sts_timestamp_to_pose_map_.end()){
        ROS_WARN("Timestamp available in LTS not found in STS.");
      }
      else{
        double x_err = it_lts->second.pose.position.x - it_sts->second.pose.position.x;
        double y_err = it_lts->second.pose.position.y - it_sts->second.pose.position.y;
        double z_err = it_lts->second.pose.position.z - it_sts->second.pose.position.z;
        double magn = sqrt(x_err*x_err + y_err*y_err + z_err*z_err);
        ROS_INFO("Error between estimators x=%f, y=%f, z=%f, magnitude=%f",x_err,y_err,z_err,magn);
      }



      ++last_found_timestamp_idx_;
    }
  }
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
