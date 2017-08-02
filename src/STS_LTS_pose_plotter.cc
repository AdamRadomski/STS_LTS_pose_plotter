#include "STS_LTS_pose_plotter.h"

#include <fstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "common.h"

PosePlotter::PosePlotter() {
  sts_msg_count_ = 0u;
  lts_msg_count_ = 0u;
  last_found_timestamp_idx_ = 0u;
  offset_ = Eigen::Vector3d( 464980.0, 5.27226e+06, 414.087);
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
  const int64_t closest_true_timestamp = findClosestTrueTimestamp(timestamp);

  Eigen::Vector3d true_pos = timestamp_to_true_pose_map_[closest_true_timestamp]-offset_;
  double x_err = msg.pose.position.x - true_pos(0);
  double y_err = msg.pose.position.y - true_pos(1);
  double z_err = msg.pose.position.z - true_pos(2);
  double magn = sqrt(x_err*x_err + y_err*y_err + z_err*z_err);
  ROS_INFO("Error of STS position: x=%f, y=%f, z=%f, magnitude=%f",x_err,y_err,z_err,magn);
}

void PosePlotter::poseCallbackLTSGlobal(const geometry_msgs::PoseStamped& msg)
{
  ++lts_msg_count_;

  const int64_t timestamp = rosTimeToNanoseconds(msg.header.stamp);
  const int64_t closest_true_timestamp = findClosestTrueTimestamp(timestamp);

  Eigen::Vector3d true_pos = timestamp_to_true_pose_map_[closest_true_timestamp]-offset_;
  double x_err = msg.pose.position.x - true_pos(0);
  double y_err = msg.pose.position.y - true_pos(1);
  double z_err = msg.pose.position.z - true_pos(2);
  double magn = sqrt(x_err*x_err + y_err*y_err + z_err*z_err);
  ROS_INFO("Error of LTS position: x=%f, y=%f, z=%f, magnitude=%f",x_err,y_err,z_err,magn);
}

int64_t PosePlotter::findClosestTrueTimestamp(const int64_t timestamp){
  int64_t closest_timestamp;

  TimestampToTruePoseMap::iterator low, prev;


  low = timestamp_to_true_pose_map_.lower_bound(timestamp);
  if (low == timestamp_to_true_pose_map_.end()) {
    // Big shit, not sure how to handle it yet.
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
    while(std::getline(test, segment, ' '))
    {
       seglist.push_back(segment);
    }
    assert(seglist.size() == 4);

    const int64_t timestamp = std::stoul(seglist[0]);
    const double x = std::stod(seglist[1]);
    const double y = std::stod(seglist[2]);
    const double z = std::stod(seglist[3]);

    // Insert the data to the map.
    timestamp_to_true_pose_map_.insert(std::make_pair(timestamp,Eigen::Vector3d(x,y,z)));
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
  plotter.readTruePoses("/home/radam/true_poses.txt");

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

  std::cout << "*** pose plotter finished ***" << std::endl;
  return 0;
}
