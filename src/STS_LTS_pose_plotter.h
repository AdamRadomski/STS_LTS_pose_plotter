#pragma once

#include <geometry_msgs/PoseStamped.h>

class PosePlotter {
 public:
  PosePlotter();

  // Callbacks called when messages are received.
  void poseCallbackSTS(const geometry_msgs::PoseStamped& msg);
  void poseCallbackLTS(const geometry_msgs::PoseStamped& msg);

 private:
  // Counters
  size_t sts_msg_count_;
  size_t lts_msg_count_;

};

// Nothing here yet

