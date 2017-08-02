#pragma once

#include "ros/time.h"

inline int64_t rosTimeToNanoseconds(const ros::Time& ros_time){
  static constexpr int64_t kSecondsToNanoseconds = 1e9;
  return (int64_t)ros_time.sec*kSecondsToNanoseconds + (int64_t)ros_time.nsec;
}

