# AntobotURCU

This This is a package for LIO. 

2025.09.02: fast-lio2 ros1 verson, loop closre with gtsam.


2025.09.10: fastlio2 antobot version. Need Sophus:1.22.10

2025.09.11: Update lio frame frome lidar -> base, now we can output Odometry with lidar. If we want get more accuracy data, we should calibrate lidar->odom->gps.

2025.09.12: Debug,the mid360 on U301 is installed upside down.Change the axis of base and lidar

2025.09.15: Add record odom msg2tum and lio2tum.

2025.09.16: Fix the lidar heading and odom cov data.

2025.09.17: Change the param of map_resolution and nearest_search to accelerate program.

2025.09.24: Update the robot_location pkg with the type of judge the status of GPS data.

2025.09.25: LIO multi threaded concurrent processing of Callback Functions.

2025.09.28: LIO convert ros1~ros2 multi thread version.

Warning: if you want to use this package in ubuntu:20.04 noetic, you should change your /usr/include/pcl-1.10/pcl/filters/voxel_grid.h in row 340 and 669 to "for (Eigen::MatrixXi::Index ni = 0; ni < relative_coordinates.cols (); ni++)",that can catkin_make right.
