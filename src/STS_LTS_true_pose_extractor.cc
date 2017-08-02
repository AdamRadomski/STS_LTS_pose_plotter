#include "STS_LTS_pose_plotter.h"

#include <Eigen/Dense>
#include <fstream>

#include <gps_common/conversions.h>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/String.h"

#include "common.h"

int main(int argc, char **argv)
{
  std::cout << "*** true pose extractor starts running ***" << std::endl;

  std::ofstream file;
  file.open("/home/radam/true_poses.txt");

  rosbag::Bag bag;
  const std::string rosbag_path_filename = "/home/radam/catkin_ws/datasets/mavros_visensor_2016-08-12-16-34-35.bag";
  try {
    bag.open(rosbag_path_filename, rosbag::bagmode::Read);
  } catch (const std::exception& ex) {  // NOLINT
    ROS_FATAL("Could not open the rosbag %s: %s", rosbag_path_filename, ex.what());
    return -1;
  }

  std::vector<std::string> topics;
  topics.push_back(std::string("/mavros/global_position/global"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for(const rosbag::MessageInstance& message : view)
  {
      const sensor_msgs::NavSatFix::Ptr msg_ptr = message.instantiate<sensor_msgs::NavSatFix>();
      if (msg_ptr != NULL) {
        sensor_msgs::NavSatFix meas = *msg_ptr;
        int64_t timestamp = rosTimeToNanoseconds(meas.header.stamp);

        double northing, easting;
        char utm_zone[10];
        const Eigen::Vector3d& latlonalt = Eigen::Vector3d(
            meas.latitude, meas.longitude,meas.altitude);

        gps_common::LLtoUTM(
            latlonalt(0), latlonalt(1), northing, easting, utm_zone);
        Eigen::Vector3d pos_utm(easting, northing, latlonalt(2));

        file << std::setprecision(15) << timestamp << " " <<  pos_utm(0)<< " " <<  pos_utm(1)<< " " <<  pos_utm(2)  << std::endl;
      }
  }

  bag.close();
  file.close();

  std::cout << "*** true pose extractor finished ***" << std::endl;
  return 0;
}







