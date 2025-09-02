# AntobotURCU

Contains 4 main packages which are necessary for any uRCU-based device:
- antobot_localisation: with the correct device configuration, can give a precise global pose (GPS location and heading)
- antobot_platform_msgs: standard messages and services used in the uRCU
- antobot_urcu: contains scripts for monitoring and managing core uRCU parameters (sofware launching, shutdown, etc.)
- antobot_lidar_odometry: as its name which privide lidar odometry for the robot, subscribe lidar data and imu data , advertise odom data.
