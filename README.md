# STS_LTS_pose_plotter
ROS node used to plot errors of STS and LTS

catkin build sts_lts_pose_plotter 

rosrun sts_lts_pose_plotter true_pose_extractor

rosrun sts_lts_pose_plotter pose_plotter

rqt_plot /error/magnitude/sts/vector/x /error/magnitude/lts/vector/x

