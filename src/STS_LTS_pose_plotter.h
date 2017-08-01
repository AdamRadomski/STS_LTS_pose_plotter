#pragma once

#include <mutex>

#include <geometry_msgs/PoseStamped.h>

#include "ros/time.h"

inline int64_t rosTimeToNanoseconds(const ros::Time& ros_time){
  static constexpr int64_t kSecondsToNanoseconds = 1e9;
  return (int64_t)ros_time.sec*kSecondsToNanoseconds + (int64_t)ros_time.nsec;
}

class PosePlotter {
 public:

  typedef std::map<int64_t,geometry_msgs::PoseStamped> TimestampToPoseMap;

  PosePlotter();

  // Callbacks called when messages are received.
  void poseCallbackSTS(const geometry_msgs::PoseStamped& msg);
  void poseCallbackLTS(const geometry_msgs::PoseStamped& msg);

 private:
  // Function calculating errors.
  void calcErrorBetweenEstimators();

  // Counters
  size_t sts_msg_count_;
  size_t lts_msg_count_;

  // Mutex protecting the buffers.
  std::mutex mutex_;

  // Maps of timestamps and corresponding messages for both estimators.
  TimestampToPoseMap sts_timestamp_to_pose_map_;
  TimestampToPoseMap lts_timestamp_to_pose_map_;

  // Set of all timestamps available in both estimators.
  std::set<int64_t> available_timestamps_;
  int64_t last_found_timestamp_idx_;

};

// Nothing here yet

