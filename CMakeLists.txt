cmake_minimum_required(VERSION 2.8.3)
project(sts_lts_pose_plotter)

## Find catkin and any catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg rosbag)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pose_plotter src/sts_lts_pose_plotter.cc)
target_link_libraries(pose_plotter ${catkin_LIBRARIES})

add_executable(true_pose_extractor src/sts_lts_true_pose_extractor.cc)
target_link_libraries(true_pose_extractor ${catkin_LIBRARIES})
