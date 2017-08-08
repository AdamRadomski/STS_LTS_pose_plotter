#pragma once

#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/time.h"

class PosePlotter {
 public:

  typedef std::map<int64_t,geometry_msgs::PoseStamped> TimestampToPoseMap;
  typedef std::map<int64_t, Eigen::MatrixXd> TimestampToTruePoseMap;

  PosePlotter();

  // Callbacks called when messages are received.
  void poseCallbackSTS(const geometry_msgs::PoseStamped& msg);
  void poseCallbackLTS(const geometry_msgs::PoseStamped& msg);

  void poseCallbackSTSGlobal(const geometry_msgs::PoseStamped& msg);
  void poseCallbackLTSGlobal(const geometry_msgs::PoseStamped& msg);

  void printAverageErrors();

  int64_t findClosestTrueTimestamp(const int64_t timestamp);
  Eigen::MatrixXd interpolateGroundTruth(const int64_t timestamp);
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

  // Vector of errors in position for both smoothers.
  std::vector<double> errors_sts_, errors_lts_;

  // Publishers for errors for both smoothers.
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_sts_, publisher_lts_;

  // Set of all timestamps available in both estimators.
  std::set<int64_t> available_timestamps_;
  int64_t last_found_timestamp_idx_;

  // Home position (offset).
  Eigen::Vector3d offset_;

};
