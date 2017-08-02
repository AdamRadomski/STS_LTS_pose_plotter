#pragma once

#include <mutex>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

#include "ros/time.h"

class PosePlotter {
 public:

  typedef std::map<int64_t,geometry_msgs::PoseStamped> TimestampToPoseMap;
  typedef std::map<int64_t, Eigen::Vector3d> TimestampToTruePoseMap;

  PosePlotter();

  // Callbacks called when messages are received.
  void poseCallbackSTS(const geometry_msgs::PoseStamped& msg);
  void poseCallbackLTS(const geometry_msgs::PoseStamped& msg);

  void readTruePoses(const std::string& file_name);

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
  TimestampToTruePoseMap timestamp_to_true_pose_map_;

  // Set of all timestamps available in both estimators.
  std::set<int64_t> available_timestamps_;
  int64_t last_found_timestamp_idx_;

};
