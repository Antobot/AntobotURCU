/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description:  The primary purpose of this code is to define the functions and variables used in heading.cpp
# Contacts: 	soyoung.kim@antobot.ai
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include <geodesy/utm.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <std_srvs/srv/trigger.h>
#include <antobot_devices_msgs/srv/progress_update.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/polygon_stamped.h>

//#include <geonav_transform/navsat_conversions.h>

//using namespace GeonavTransform::NavsatConversions;

//namespace heading
//{
class AntobotHeading : public rclcpp::Node
{
    public:
        AntobotHeading() : Node("antobot_heading"), count_(0){};
        ~AntobotHeading();
        void initialCalibration(void);
    protected:
		//ros::NodeHandle nh_;
        tf2::Matrix3x3 mat;
        size_t count_;
        
        bool dual_gps;
        bool heading_received;
        std_msgs::msg::Float64 dual_gps_heading;
        // configuration parameters
        double calib_distance, angular_zero_tol, lin_tol, calib_deg;
        // GPS
        std::vector<double> gps_start, gps_end;
        std::string gps_frame;
        bool gps_received;
        double utm_y, utm_x, rtk_status;
        std::string utm_zone;
        int rtk_target_status; 
        // imu
        geometry_msgs::msg::Quaternion q_imu;
        std::string imu_frame;
        bool imu_received;
        float imu_ang_vel_z;
        double imu_offset;
        int imu_calibration_status;
        // odometry
        geometry_msgs::msg::Quaternion q_odom;
        bool odometry_received;
        // wheel odometry
        double wheel_odom_v;
        // actual robot cmd_vel
        geometry_msgs::msg::Twist robot_cmd_vel;
        // costmap
        bool costmap_received;
        std::vector<std::vector<int>> costmap_msg;
        int width,height;
        float resolution;
        float footprint_x, footprint_y;
        bool footprint_received;
        
        // Other parameters
        bool direction; // true if the robot is moving forward, false if the robot is moving backward
        double gps_yaw;
        bool hmi_auto_button_pressed;
        rclcpp::Time hmi_auto_button_time, heading_received_time;
        // Read parameters to set the topic names
        std::string gps_topic, imu_topic, odometry_topic, wheelodometry_topic;
        bool sim;
        
        // Subscriber
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;  
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_odom;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_footprint;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_heading;
        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_imu_offset; 
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_calib;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_imu_z;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_GPS_origin; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_switch;
        
        // Timer
        //rclcpp::TimerBase auto_calibration_timer;
        //rclcpp::TimerBase imu_pub_timer;
        
        // Clients
        //rclcpp::Client<std_srvs::srv::Trigger> ekf_srv;
        //rclcpp::Client<antobot_devices_msgs::srv::ProgressUpdate> hmi_srv;
        
        // Services
        //rclcpp::Service<antobot_devices_msgs::srv::ProgressUpdate> hmi_calibration_srv;
        
        // Functions
        void initialise(void);
        std::vector<double> eulerFromQuat(tf2::Quaternion);
        tf2::Quaternion quatFromEuler(double a, double b, double c);
        //void autoCalibrate(const rclcpp::TimerEvent& event);
        //void publishNewIMU(const rclcpp::TimerEvent& event);
        bool saveStartGPS(void);
        int checkCondition(void);
        void checkInputs(void);
        double calculateDifference(double, double);
        double normalize_angle(double);
        double autoCalibration_hmi(void);
        bool hmiService(antobot_devices_msgs::srv::ProgressUpdate::Request &req, antobot_devices_msgs::srv::ProgressUpdate::Response &res );
        bool checkObstacle(void);
        bool dualGPSHeadingCalibration(void);
        // Callback functions 
        void gpsCallback(const sensor_msgs::msg::NavSatFix &msg);
        void headingCallback(const std_msgs::msg::Float64 &msg); // dual GPS
        void imuCallback(const sensor_msgs::msg::Imu &msg);
        void odometryCallback(const nav_msgs::msg::Odometry &msg);
        void wheelOdomCallback(const nav_msgs::msg::Odometry &msg);
        void costmapCallback(const nav_msgs::msg::OccupancyGrid &msg);
        void footprintCallback(const geometry_msgs::msg::PolygonStamped &msg);
        void cmdVelCallback(const geometry_msgs::msg::Twist &msg);
};
//}