#include "sts_lts_pose_plotter.h"

#include <functional>
#include <fstream>
#include <numeric>
#include <string>

#include <gps_common/conversions.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "common.h"

PosePlotter::PosePlotter() {
  sts_msg_count_ = 0u;
  lts_msg_count_ = 0u;
  last_found_timestamp_idx_ = 0u;
  offset_ = Eigen::Vector3d( 464980.0, 5.27226e+06, 414.087);

  publisher_pose_error_sts_ = node_handle_.advertise<geometry_msgs::Vector3Stamped>("/error/pose/sts", 1);
  publisher_pose_error_lts_ = node_handle_.advertise<geometry_msgs::Vector3Stamped>("/error/pose/lts", 1);
  publisher_orientation_error_sts_ = node_handle_.advertise<geometry_msgs::Vector3Stamped>("/error/orientation/sts", 1);
  publisher_orientation_error_lts_ = node_handle_.advertise<geometry_msgs::Vector3Stamped>("/error/orientation/lts", 1);
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

void PosePlotter::poseCallbackSTSGlobal(const geometry_msgs::PoseStamped& msg)
{
  ++sts_msg_count_;

  const int64_t timestamp = rosTimeToNanoseconds(msg.header.stamp);

  const Eigen::MatrixXd true_pos = interpolateGroundTruth(timestamp) - offset_;
  const double x_err = msg.pose.position.x - true_pos(0,0);
  const double y_err = msg.pose.position.y - true_pos(1,0);
  const double z_err = msg.pose.position.z - true_pos(2,0);
  const double magn = sqrt(x_err*x_err + y_err*y_err + z_err*z_err);
  errors_sts_.push_back(magn);
  //ROS_INFO("Error of STS position: x=%f, y=%f, z=%f, magnitude=%f",x_err,y_err,z_err,magn);

  geometry_msgs::Vector3Stamped error_msg;
  static constexpr int64_t kSecondToNanosecond = 1e9;
  error_msg.header.stamp = ros::Time(timestamp / kSecondToNanosecond, timestamp % kSecondToNanosecond);
  error_msg.vector.x = x_err;
  error_msg.vector.y = y_err;
  error_msg.vector.z = z_err;
  publisher_pose_error_sts_.publish(error_msg);
}

void PosePlotter::poseCallbackLTSGlobal(const geometry_msgs::PoseStamped& msg)
{
  ++lts_msg_count_;

  const int64_t timestamp = rosTimeToNanoseconds(msg.header.stamp);

  const Eigen::MatrixXd true_pos = interpolateGroundTruth(timestamp) - offset_;
  const double x_err = msg.pose.position.x - true_pos(0,0);
  const double y_err = msg.pose.position.y - true_pos(1,0);
  const double z_err = msg.pose.position.z - true_pos(2,0);
  const double magn = sqrt(x_err*x_err + y_err*y_err + z_err*z_err);
  errors_lts_.push_back(magn);
  //ROS_INFO("Error of LTS position: x=%f, y=%f, z=%f, magnitude=%f",x_err,y_err,z_err,magn);

  geometry_msgs::Vector3Stamped error_msg;
  static constexpr int64_t kSecondToNanosecond = 1e9;
  error_msg.header.stamp = ros::Time(timestamp / kSecondToNanosecond, timestamp % kSecondToNanosecond);
  error_msg.vector.x = x_err;
  error_msg.vector.y = y_err;
  error_msg.vector.z = z_err;
  publisher_pose_error_lts_.publish(error_msg);
}

void PosePlotter::printAverageErrors(){
  // Print average errors for both estimators.
  double average_error_sts = std::accumulate( errors_sts_.begin(), errors_sts_.end(), 0.0)/errors_sts_.size();
  double average_error_lts = std::accumulate( errors_lts_.begin(), errors_lts_.end(), 0.0)/errors_lts_.size();

  std::cout << "Average error for STS=" << average_error_sts << ", LTS=" <<average_error_lts << std::endl;
}

int64_t PosePlotter::findClosestTrueTimestamp(const int64_t timestamp){
  int64_t closest_timestamp;

  TimestampToTruePoseMap::iterator low, prev;

  low = timestamp_to_true_pose_map_.lower_bound(timestamp);
  if (low == timestamp_to_true_pose_map_.end()) {
    // Problem here, not sure how to handle it yet.
    assert(false);

  } else if (low == timestamp_to_true_pose_map_.begin()) {
      closest_timestamp = low->first;
    } else {
      prev = low;
      --prev;
      if ((timestamp - prev->first) < (low->first - timestamp))
        closest_timestamp = prev->first;
      else
        closest_timestamp = low->first;
  }
  return closest_timestamp;
}

Eigen::MatrixXd PosePlotter::interpolateGroundTruth(const int64_t timestamp){
  // Find timestamps before and after the given timestamp.
  TimestampToTruePoseMap::iterator prev, next;
  next = timestamp_to_true_pose_map_.lower_bound(timestamp);
  prev = next;
  prev--;

  if (next == timestamp_to_true_pose_map_.end() ) {
    // Problem here, not sure how to handle it yet.
    assert(false);
  }
  else if (next->first == timestamp){
    // Exactly at the timestamp.
    std::cout << "return 1 " <<prev->first << " " << timestamp << " " <<next->first << std::endl;
    return next->second;
  }
  else {
    const Eigen::MatrixXd& prev_mat = prev->second;
    const Eigen::MatrixXd& next_mat = next->second;
    const int64_t& prev_timestamp = prev->first;
    const int64_t& next_timestamp = next->first;

    Eigen::MatrixXd a = (next_mat - prev_mat)/((double)(next_timestamp - prev_timestamp));
    Eigen::MatrixXd b = prev_mat - a*prev_timestamp;
    return a*timestamp + b;
  }
}

void PosePlotter::readTruePoses(const std::string& file_name){
  std::ifstream file;
  file.open(file_name);
  assert(file);

  std::string line;
  while (getline( file, line ))
  {
    // Split the line on spacebars
    std::stringstream test(line);
    std::string segment;
    std::vector<std::string> seglist;
    while(std::getline(test, segment, '\t'))
    {
       seglist.push_back(segment);
    }
    assert(seglist.size() == 7);

    if (seglist.size() == 0 ){
      break;
    }

    const int64_t timestamp = std::stoul(seglist[0]);
    const double lon = std::stod(seglist[1]);
    const double lat = std::stod(seglist[2]);
    const double alt = std::stod(seglist[3]);
    const double yaw = std::stod(seglist[4]);
    const double pitch = std::stod(seglist[5]);
    const double roll = std::stod(seglist[6]);

    double northing, easting;
    char utm_zone[10];
    const Eigen::Vector3d& latlonalt = Eigen::Vector3d(
        lat, lon,alt);

    gps_common::LLtoUTM(
        latlonalt(0), latlonalt(1), northing, easting, utm_zone);
    Eigen::Vector3d pos_utm(easting, northing, latlonalt(2));

    // Insert the data to the map.
    Eigen::MatrixXd matrix;
    matrix.resize(6,1);
    matrix << pos_utm(0), pos_utm(1), pos_utm(2), yaw, pitch, roll;
    timestamp_to_true_pose_map_.insert(std::make_pair(timestamp,matrix));
    //std::cout << std::setprecision(15) <<  pos_utm(0) << " " <<  pos_utm(1)<< " " <<  pos_utm(2)<< " " <<  yaw<< " " <<  pitch<< " " <<  roll << std::endl;
  }
  file.close();
  std::cout << "*** finished reading input file *** " << std::endl;
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

  // Comment it out to calculate relative errors
#define CALCULATE_GLOBAL_ERROR

  // Prepare the plotter.
  PosePlotter plotter;
  plotter.readTruePoses("/home/radam/true poses/true_poses_pix4d.txt");

  // Subscribe to nodes.
  ros::NodeHandle n;
#ifdef CALCULATE_GLOBAL_ERROR
    // Calculate global errors, separately for each smoother.
  ros::Subscriber sub_sts = n.subscribe("/fw_pose/sts", 1000, &PosePlotter::poseCallbackSTSGlobal, &plotter);
  ros::Subscriber sub_lts = n.subscribe("/fw_pose/lts", 1000, &PosePlotter::poseCallbackLTSGlobal, &plotter);
#else
    // Calculate errors between smoothers.
    ros::Subscriber sub_sts = n.subscribe("/fw_pose/sts", 1000, &PosePlotter::poseCallbackSTS, &plotter);
    ros::Subscriber sub_lts = n.subscribe("/fw_pose/lts", 1000, &PosePlotter::poseCallbackLTS, &plotter);
#endif

  ros::spin();

  plotter.printAverageErrors();
  std::cout << "*** pose plotter finished ***" << std::endl;
  return 0;
}
