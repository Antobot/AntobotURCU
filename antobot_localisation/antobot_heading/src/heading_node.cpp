/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: Launches heading_node and runs the initial calibration.
# Contacts: soyoung.kim@antobot.ai
# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

//#include <antobot_heading/heading.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
//#include <geodesy/utm.h>
//#include <geographic_msgs/msg/geo_point.hpp>
#include <navsat_conversions.h>
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
#include "std_srvs/srv/trigger.hpp"
#include <antobot_devices_msgs/srv/progress_update.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/polygon_stamped.h>

using std::placeholders::_1;

//#include <geonav_transform/navsat_conversions.h>

//using namespace GeonavTransform::NavsatConversions;

//namespace heading
//{
class AntobotHeading : public rclcpp::Node
{
    public:
      AntobotHeading() : Node("antobot_heading"), count_(0)
      {   
          // Description: Constructor for the heading class
          initialise();
          initialCalibration();
      }
      //~AntobotHeading();
      //void initialCalibration(void);
      
    private:
		    //ros::NodeHandle nh_;

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr nh_global_;
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
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_calib;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_imu_z;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_GPS_origin; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_switch;
        
        // Timer
        rclcpp::CallbackGroup::SharedPtr callback_group;
        rclcpp::TimerBase::SharedPtr auto_calibration_timer_;
        rclcpp::TimerBase::SharedPtr imu_pub_timer_;
        
        // Clients
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ekf_cli;
        rclcpp::Client<antobot_devices_msgs::srv::ProgressUpdate>::SharedPtr hmi_cli;
        
        // Services
        rclcpp::Service<antobot_devices_msgs::srv::ProgressUpdate>::SharedPtr hmi_calibration_srv;
        
        // Functions
        void initialise(){
          // Description: Initialises autoCalibration class
          /*if (nh_.hasParam("/gps/ublox_rover")){
              RCLCPP_INFO(this->get_logger(), "MV2100: Heading IMU calibration - using DUAL GPS");
              dual_gps = true;
          }
          else{
              RCLCPP_INFO(this->get_logger(), "MV2100: Heading IMU calibration - using One GPS & wheel odometry");
              dual_gps = false;
          }*/

          // configuration parameters
          calib_distance = 1.0; // calibration distance 
          angular_zero_tol = 0.1; // Angular velocity (rad/sec)
          lin_tol = 0.01; // linear velocity x (m/s)
          calib_deg = 3; // deg
          
          // GPS
          gps_frame = "gps_frame";
          gps_start.resize(2);
          gps_end.resize(2);
          gps_received = false;
          utm_y = 0;
          utm_x = 0;
          utm_zone = "";
          rtk_status = -1; // default value
          
          // dual GPS
          heading_received = false;
          heading_received_time = this->now();
          
          // imu
          imu_frame = "imu_frame";
          imu_received = false;
          imu_ang_vel_z = 0;
          imu_offset = 0;
          imu_calibration_status = -1;
          
          // odometry
          odometry_received = false;
          
          // wheel odometry
          wheel_odom_v = 0;
          robot_cmd_vel = geometry_msgs::msg::Twist();
          
          //costmap
          costmap_received = false;
          
          // Other parameters
          direction = true; // true if the robot is moving forward, false if the robot is moving backward
          gps_yaw = 0;
          hmi_auto_button_pressed = false;
          hmi_auto_button_time =this->now();
          
          // Read parameters to set the topic names - need new method for ROS2!
          //nh_.getParam("/heading_node/gps_topic", gps_topic);
          gps_topic = "antobot_gps";
          this->declare_parameter("/heading_node/gps_topic", gps_topic);
          //nh_.getParam("/heading_node/imu_topic", imu_topic);
          imu_topic = "/imu/data";
          this->declare_parameter("heading_node/imu_topic", imu_topic);
          //nh_.getParam("/heading_node/odometry_topic", odometry_topic);
          odometry_topic = "/odometry/filtered";
          this->declare_parameter("heading_node/odometry_topic", odometry_topic);
          //nh_.getParam("/heading_node/wheel_odometry_topic", wheelodometry_topic);
          wheelodometry_topic = "/antobot/robot/odometry";
          this->declare_parameter("heading_node/wheel_odometry_topic", wheelodometry_topic);
          //nh_.param<bool>("/simulation", sim, false);

          sim = true;
          rtk_target_status = 3; // rtk status is 3 in 3D fixed mode
          if (sim){
              rtk_target_status = 0; // in simulation, gps status is always 0
          }
          // Subscribers
          sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, 10, std::bind(&AntobotHeading::gpsCallback, this, _1));
          sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&AntobotHeading::imuCallback,this, _1));
          sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10, std::bind(&AntobotHeading::odometryCallback,this,_1));
          sub_wheel_odom = this->create_subscription<nav_msgs::msg::Odometry>(wheelodometry_topic, 10, std::bind(&AntobotHeading::wheelOdomCallback,this,_1));
          sub_costmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap_node/costmap/costmap", 10, std::bind(&AntobotHeading::costmapCallback,this,_1));
          sub_footprint = this->create_subscription<geometry_msgs::msg::PolygonStamped>("/costmap_node/costmap/footprint", 10, std::bind(&AntobotHeading::footprintCallback,this,_1));
          sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("/antobot/robot/cmd_vel", 10, std::bind(&AntobotHeading::cmdVelCallback,this, _1));
          sub_heading = this->create_subscription<std_msgs::msg::Float64>("am_heading_robot", 10, std::bind(&AntobotHeading::headingCallback,this,_1));
          
          // Publisher
          pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_corrected", 10);
          pub_imu_z = this->create_publisher<std_msgs::msg::Float32>("/imu/heading_z", 10); // For opcua
          pub_imu_offset = this->create_publisher<std_msgs::msg::Float32>("/imu/data_offset", 10); // For debug
          pub_calib = this->create_publisher<std_msgs::msg::UInt8>("/imu_calibration_status", 10); // For heading_launcher. Notify when the initial calibraion is done.
          pub_GPS_origin = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps_map_origin",10);
          pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/am_nav/cmd_vel",10); // cmd_vel for HMI auto calibration
          pub_switch = this->create_publisher<std_msgs::msg::UInt8>("/antobot/teleop/switch_mode", 10);
          
          // Clients
          ekf_cli = this->create_client<std_srvs::srv::Trigger>("launch_ekf"); // launch ekf nodes once the initial calibration is done
          hmi_cli = this->create_client<antobot_devices_msgs::srv::ProgressUpdate>("/calibration_HMI/progressUpdate"); // update progress in HMI bridge
          
          // Service - currently causes large build error!
          hmi_calibration_srv = this->create_service<antobot_devices_msgs::srv::ProgressUpdate>("/calibration/progressUpdate", 
            std::bind(&AntobotHeading::hmiService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
          
          // Timer
          callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
          auto_calibration_timer_ = this->create_wall_timer(std::chrono::seconds(15), std::bind(&AntobotHeading::autoCalibrate, this), callback_group); // every 15 secs # was 30 sec
          imu_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AntobotHeading::publishNewIMU, this), callback_group); // 20Hz 
        }

        
        //void autoCalibrate(rclcpp::Node::SharedPtr nh_){
        void autoCalibrate(){
           // Description: Function that is called every 30 seconds to check if the calibration is needed. 
           // In the while loop, several conditions are checked and if theses conditions are met within 20 seconds, the new imu_offset is calculated (calibration)
            RCLCPP_INFO(this->get_logger(), "AutoCalibration called");
            nh_global_ = this->get_node_base_interface();
          
            rclcpp::Time started_time =this->now();
            if (imu_calibration_status == 1 && odometry_received){
               RCLCPP_INFO(this->get_logger(),"auto calibration called but skip one time"); // Since the initial calibration was done recently
               imu_calibration_status = 2;
            }  
            else if (imu_calibration_status == 2){
                RCLCPP_INFO(this->get_logger(),"auto calibration checking started");
                int state = 0;
                while(true){
                    //rclcpp::spin_some(nh_global_);
                    if (state == 0){ // Check rtk status and ekf odometry
                        //if (rtk_status == rtk_target_status && odometry_received){
                        if (rtk_status == rtk_target_status && odometry_received){   //1 = float; 3 = fix
                            if (dual_gps){ // dual GPS doesn't require calculating the heading based on the GPS position
                                if (dualGPSHeadingCalibration()){
                                    state = 3;
                                }  // if dual gps heading is not updated - state remains 0
                                else{
                                    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // TODO: whether 1s is too long?
                                }
                              
                            }
                            else{
                                state = 1;
                            }
                        }
                    }
                    else if (state == 1){ // Save start gps position
                        if (saveStartGPS()){ // If the first gps position is saved
                            state = 2;
                        }
                    }
                    else if (state == 2){ // Check if the auto calibration conditions are met
                        state= checkCondition(); // returns the state
                    }
                    else if (state == 3){ // Conditions met, start the calibration
                        // gps angle calculated 
                        RCLCPP_DEBUG(this->get_logger(), "gps angles = %f", gps_yaw);
                        tf2::Quaternion quat_tf_imu;
                        tf2::convert(q_imu , quat_tf_imu);
                        std::vector<double> result_imu = eulerFromQuat(quat_tf_imu); // returns r,p,y
                        tf2::Quaternion quat_tf_odom;
                        tf2::convert(q_odom , quat_tf_odom);
                        std::vector<double> result_odom = eulerFromQuat(quat_tf_odom);
                        // imu_yaw + imu_offset = gps_yaw
                        // Compare two angles in [-pi,pi] and returns signed value in radian
                        double diff = calculateDifference(gps_yaw,result_odom[2]);
                        RCLCPP_DEBUG(this->get_logger(), "diff = %f", diff);
                        double diff_deg = abs(diff*180.0/M_PI);
                        RCLCPP_DEBUG(this->get_logger(), "angle diff = %f",diff_deg);
                        if (diff_deg > calib_deg){
                            double imu_offset_tmp = calculateDifference(gps_yaw, result_imu[2]); // difference between orientations from imu and gps
                            imu_offset = imu_offset_tmp;
                            RCLCPP_INFO(this->get_logger(), "MV2200: Auto-calibration complete (imu offset = %f degs)", imu_offset / M_PI * 180.0);
                        }
                        else{
                            RCLCPP_INFO(this->get_logger(), "auto-calibration not required %f deg",diff_deg);
                        }
                        break; // Auto calibration finished
                    }
                    double duration = (this->now()- started_time).seconds();
                    if (duration > 10.0){ // Every time Auto calibration is called, it will check for 10 seconds. #was 20 sec
                        RCLCPP_INFO(this->get_logger(), "auto calibration conditions not met - cancelled");
                        break;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                }
            }
            else{
                RCLCPP_INFO(this->get_logger(), "auto calibration called but ignored - Do initial calibration first!");
            }
        }

        void publishNewIMU(){
            // Description: Publish calibrated imu data (/imu/dadta_coreccted) and a topic for debugging purpose (/imu/data_offset)
            // Input: imu raw data and calculated imu_offset. imu_offset is added to the yaw value of the imu data. 
            if (imu_calibration_status > 0){
                tf2::Quaternion quat_tf;
                geometry_msgs::msg::Quaternion quat_msg;
                // Convert  ROS Quaternion to tf2 Quaternion msg
                tf2::convert(q_imu , quat_tf);
                // Convert from Quaternion to euler
                std::vector<double> result = eulerFromQuat(quat_tf);
                // Add imu offset and convert back to Quaternion  
                tf2::Quaternion result_tf =  quatFromEuler(result[0],result[1],result[2]+imu_offset);
                std_msgs::msg::Float32 heading_z_msg;
                heading_z_msg.data = result[2]+imu_offset;
                pub_imu_z->publish(heading_z_msg);
                // Convert tf2 Quaternion to ROS Quaternion
                tf2::convert(result_tf ,quat_msg);
                
                // create imu message
                sensor_msgs::msg::Imu imuMsg;
                imuMsg.header.stamp = this->now();
                imuMsg.header.frame_id = imu_frame;
                imuMsg.orientation.x = quat_msg.x;
                imuMsg.orientation.y = quat_msg.y;
                imuMsg.orientation.z = quat_msg.z;
                imuMsg.orientation.w = quat_msg.w;
                
                imuMsg.angular_velocity.z = imu_ang_vel_z;  // for EKF inupt
                // publish /imu/data_corrected
                pub_imu->publish(imuMsg);
                // Publsih /imu/data_offset
                std_msgs::msg::Float32 imu_offset_msg;
                imu_offset_msg.data = (float)imu_offset;
                pub_imu_offset->publish(imu_offset_msg);
            }       
        }


        std::vector<double> eulerFromQuat(tf2::Quaternion q)
        {
          // Description: Convert tf2::Quaternion to euler and return vector<double> that is filled with euler values. 
          // Input: tf2::Quaternion q
          // Output: vector with euler value - roll, pitch, yaw 
          tf2::Matrix3x3 m(q);
          double r,p,y;
          m.getRPY(r,p,y);
          std::vector<double> v;
          v.push_back(r);
          v.push_back(p);
          v.push_back(y);
          return v; 
        }

        tf2::Quaternion quatFromEuler(double a, double b, double c)
        {
          // Description: Convert euler values to tf2::Quaternion 
          // Input: euler value - roll, pitch, yaw 
          // Returns: tf2::Quaternion
          tf2::Quaternion q;
          q.setRPY(a,b,c);
          q =q.normalize();
          return q;
        }

        bool saveStartGPS(void)
        {
          // Description: Save gps point as the starting point if the robot is moving. Set direction depending on the linear x velocity.
          // Robot wheel odometry's linear x should be larger than lin_tol and the imu angular z velocity should be smaller than angular_zero_tol.
          // direction is set to true when the robot is moving forward.
          // Returns: True if the start GPS point is saved, False if the robot velcoity didn't meet the conditons and failed to save the start GPS point.
          double vel = wheel_odom_v;
          if ((abs(imu_ang_vel_z) < angular_zero_tol) && (abs(vel) > lin_tol)&& (abs(robot_cmd_vel.linear.x)>lin_tol)){
              RCLCPP_INFO(this->get_logger(), "Saving starting GPS");
              gps_start.clear();
              gps_start.push_back(utm_x);
              gps_start.push_back(utm_y); // utm zone is not used for calculation - check needed
              if (vel > 0){
                  direction = true; // forward
              }
              else{
                  direction = false; // backward
              }
              return true;
          }   
          else{
              return false;
          }
        }

        int checkCondition(void)
        {
          // Description: Check several conditions in order to start calibration and returns the state that will be used in the while loop that called this function.
          // Condition: 1. imu angular z velocity should be smaller than angular_zero_tol.
          //            2. Robot's direction (forward/backward) should be consistent and hasn't change since the saving of the staring GPS point.
          //            3. Robot has moved more than calib_distance (calculated based on gps lat/long data)
          // Returns: state that will be used in the while loop that called this function
          //          state 1: Reset and save another GPS start point
          //          state 2: Distance moved is not enough, continue to call checkCondition
          //          state 3: Distance moved is larger than calib_distance, now exit the while loop and use the gps_yaw in the next step of the calibration.
          // Reset if angular vel is large
          if (abs(imu_ang_vel_z) > angular_zero_tol){
            RCLCPP_DEBUG(this->get_logger(), "Reset due to angular velocity");
            return 1;
          }
          bool current_dir; 
          // Reset if direction is changed
          if (wheel_odom_v > lin_tol){
              current_dir = true; // forward
          }
          else if (abs(wheel_odom_v) <= lin_tol){
              current_dir = direction; // very small velocity - hasn't changed the direction
          } 
          else{
              current_dir = false; // backward
          }
              
          if (current_dir != direction){
              RCLCPP_DEBUG(this->get_logger(), "Reset due to change in direction");
              return 1;
          }
          // Check the distance
          gps_end.clear();
          gps_end.push_back(utm_x);
          gps_end.push_back(utm_y); 
          double dx, dy, d;
          if (direction){ // forward
              dy = gps_end[1] - gps_start[1];
              dx = gps_end[0] - gps_start[0];
          } 
          else{ // backward
              dy = gps_start[1] - gps_end[1];
              dx = gps_start[0] - gps_end[0];
          }     
          d = sqrt(dx*dx + dy*dy);   // distance between start and end points of calibration
          gps_yaw = atan2(dy, dx);   // calculate yaw from gps start and end points
          if (d >= calib_distance){
              RCLCPP_DEBUG(this->get_logger(),"calibration distance satisfied");
              return 3;
          }
          else{
              return 2; // not satisfied
          }
        }

        void initialCalibration()
        {
            // Description: Function that is called once in the main function. When all the required topics are recieved (self.checkInputs)
            // this function performs the initial calibration. In the while loop, the starting gps postion is saved and when the 
            // Calibration conditions are met, the imu_offset is calculated. 
            // Check if all input toics are being published
            
            nh_global_ = this->get_node_base_interface();

            checkInputs();
            int state = 1;
            
            // update HMI progress : state 1 - ready for calibration
            auto hmi_req = std::make_shared<antobot_devices_msgs::srv::ProgressUpdate::Request>();
            hmi_req->progress_code = 1;
            auto hmi_res1 = hmi_cli->async_send_request(hmi_req);
            RCLCPP_INFO(this->get_logger(), "HMI progress updated - state 1 (ready for calibration)");
            if (dual_gps){
                rclcpp::sleep_for(std::chrono::milliseconds(200)); // prevent instant HMI screen change
                RCLCPP_INFO(this->get_logger(), "Heading calibration using dual gps");
                while (dualGPSHeadingCalibration()== false){
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                    rclcpp::spin_some(nh_global_); // Initial Calibration runs before spin() in main loop
                };
                state = 3; 
            }
            
            while (true){
                rclcpp::spin_some(nh_global_);
                //RCLCPP_INFO(this->get_logger(), "Robot in state %d", state);
                if (! dual_gps && hmi_auto_button_pressed && (state != 3)){ // to prevent a bug that makes robot drive 1m more after finishing calibration - possibly due to hmi button being pressed for a longer time. 
                    hmi_auto_button_pressed = false; 
                    // update HMI progress : state 2 - In auto calibration
                    auto hmi_req = std::make_shared<antobot_devices_msgs::srv::ProgressUpdate::Request>();
                    hmi_req->progress_code = 2;
                    auto result = hmi_cli->async_send_request(hmi_req);
                    RCLCPP_INFO(this->get_logger(), "HMI progress updated - state 2 (In auto calibration) - Robot will move forward");
                    state = 4;
                } // If moving for set distance fail - send fail progress 4
                if (state == 1){
                    // moving forward/backward, no angular : set start gps point
                    if (saveStartGPS()){
                        state = 2;
                    }
                }
                else if (state == 2){
                    state = checkCondition(); // returns the state
                }
                else if (state == 3){ // calibration satisfied
                    sensor_msgs::msg::NavSatFix gpsMsg;
                    gpsMsg.header.stamp = this->now();
                    gpsMsg.header.frame_id = gps_frame;
                    GeonavTransform::NavsatConversions::UTMtoLL(gps_end[1],gps_end[0],utm_zone,gpsMsg.latitude,gpsMsg.longitude);
                    
                    this->declare_parameter("/GPS_origin/latitude", gpsMsg.latitude);
                    this->declare_parameter("/GPS_origin/longitude", gpsMsg.longitude);
                    break;
                }
                else if (state ==4){ // auto calibration
                    state = autoCalibration_hmi(); // return 3 when success, return to 1 when failed
                }
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
            RCLCPP_DEBUG(this->get_logger(), "gps_yaw is %f", gps_yaw); 
            rclcpp::spin_some(nh_global_);
            tf2::Quaternion quat_tf;
            // Convert  ROS Quaternion to tf2 Quaternion msg
            tf2::convert(q_imu , quat_tf);
            // Convert from Quaternion to euler
            std::vector<double> euler_angles = eulerFromQuat(quat_tf); // result[2] is yaw angle
            // imu_yaw + imu_offset = gps_yaw
            imu_offset = calculateDifference(gps_yaw, euler_angles[2]); // difference between orientations from imu and gps
            // ROS_DEBUG("calibration successful (offset = %f degs)", imu_offset/M_PI*180.0); // wrt map x frame
            // Let heading launcher know the inital calibration is finished 
            imu_calibration_status = 1;
            std_msgs::msg::UInt8 msg;
            msg.data = imu_calibration_status;
            pub_calib->publish(msg);
            RCLCPP_INFO(this->get_logger(), "MV2100: Initial calibration finished, (offset = %f degs)", imu_offset/M_PI*180.0);
            // update HMI progress : state 3 - calibration success
            hmi_req->progress_code = 3;
            auto hmi_res2 = hmi_cli->async_send_request(hmi_req);
            RCLCPP_INFO(this->get_logger(), "HMI progress updated - state 3 (ready for job)");
            auto ekf_req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto ekf_res1 = ekf_cli->async_send_request(ekf_req);
        }
        
        void checkInputs()
        {
            //nh_global_ = this->get_node_base_interface();

            // Description: Check if the imu, gps data are recieved and check if the RTK status is fixed mode value

            auto start = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "Waiting for initial imu and gps data - new");
            while ((!imu_received) || (!gps_received)){

                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
                if (elapsed > 5){
                    start = std::chrono::steady_clock::now();
                    RCLCPP_INFO(this->get_logger(), "Waiting for initial imu and gps data - new");
                }
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                rclcpp::spin_some(nh_global_); // Initial Calibration runs before spin() in main loop
                                        // spin_some may need to be replaced by spin_all
            }
                
            RCLCPP_INFO(this->get_logger(), "IMU and gps value received, check RTK status");

            start = std::chrono::steady_clock::now();
            while (rtk_status != rtk_target_status){
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

                if (elapsed > 5){
                    start = std::chrono::steady_clock::now();
                    RCLCPP_INFO(this->get_logger(), "Waiting for RTK status to become %d",rtk_target_status);
                }

                rclcpp::sleep_for(std::chrono::milliseconds(100));
                rclcpp::spin_some(nh_global_);
            }
                
            // RCLCPP_INFO(this->get_logger(), "RTK status is %d",rtk_target_status);
            start = std::chrono::steady_clock::now();
            while (dual_gps && !heading_received){
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
                if (elapsed > 5){
                    RCLCPP_INFO(this->get_logger(), "Waiting for dual GPS heading to be received");
                }
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
                rclcpp::spin_some(nh_global_);
            }
                
            RCLCPP_INFO(this->get_logger(), "GPS, IMU, RTK all ready - start calibration");
        }

        double calculateDifference(double input_a, double input_b)
        {
          // Description: Compare two values in [-pi,pi] and return signed value in the same range
          // Input: Two double values in [-pi,pi]
          // Returns: Difference between two input values in [-pi,pi] 
          double diff = input_a - input_b;
          return normalize_angle(diff);
        }

        double normalize_angle(double angle)
        {
          angle = fmod(angle + M_PI, 2 * M_PI);  // Wrap within the range [0, 2*pi)
          if (angle < 0) {
              angle += 2 * M_PI; // Make sure it’s in [0, 2*pi]
          }
          return angle - M_PI;  // Shift to the range [-pi, pi]
        }

        double autoCalibration_hmi()
        {
          // auto calibration triggered by HMI button
    
          // save start GPS
          gps_start.clear();
          gps_start.push_back(utm_x);
          gps_start.push_back(utm_y); 
          RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: start GPS saved");
          geometry_msgs::msg::Twist tmp;
          std_msgs::msg::UInt8 switch_tmp;
          switch_tmp.data = 3;
          pub_switch->publish(switch_tmp);
          RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: Change switch mode to autonomous mode");
          
          bool success = false;
          RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: Enter while loop (Try auto calibration for 15secs)");

          nh_global_ = this->get_node_base_interface();

          while (true){
          
              double duration = (this->now() - hmi_auto_button_time).seconds();
              if (duration > 15){
                  RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: Time out (15s)- Fail");
                  // update HMI progress : state 4 - calibration fail
                  //antobot_devices_msgs::srv::ProgressUpdate hmi_req;
                  auto hmi_req = std::make_shared<antobot_devices_msgs::srv::ProgressUpdate::Request>();
                  hmi_req->progress_code = 4;
                  auto result = hmi_cli->async_send_request(hmi_req);
                  RCLCPP_INFO(this->get_logger(), "HMI progress updated - state 4 (auto calibration fail)");
                  break;
              }
              // TODO: Fail if joystick/teleop is triggered - back to state 1 & pub 4
              rclcpp::spin_some(nh_global_); 
              // check obstacle - if True, don't publish cmd_vel 
              if (checkObstacle()){
                  RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: Obstacle found - Cmd vel 0.0");
                  tmp.linear.x = 0.0; 
              }
              else{
                  if (!costmap_received){
                      RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: costmap not received! - Cmd vel 0.0");
                      tmp.linear.x = 0.0;
                  }
                  else{
                      RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: Obstacle Not found - Cmd vel 0.3");
                      tmp.linear.x = 0.3;
                  }
                  
              }
              pub_cmd_vel->publish(tmp);   
              rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
              rclcpp::spin_some(nh_global_);

              // Check the distance
              gps_end.clear();
              gps_end.push_back(utm_x);
              gps_end.push_back(utm_y); 
              
              double dx, dy, d;
              dy = gps_end[1] - gps_start[1];
              dx = gps_end[0] - gps_start[0];
              d = sqrt(dx*dx + dy*dy);   // distance between start and end points of calibration
              
              if (d >= calib_distance){
                  RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: calibration distance satisfied (1m)");
                  gps_yaw = atan2(dy, dx);   // calculate yaw from gps start and end points
                  success = true;
                  tmp.linear.x = 0.0;
                  pub_cmd_vel->publish(tmp);
                  break;
              }
              else{
                  RCLCPP_INFO(this->get_logger(), "HMI auto-calibration: calibration distance(1m) Not satisfied %f",d);
              }
          }
          if (success){
              return 3; // success
          }
          else{
              return 1; // back to default mode
          }
        }

        void hmiService(
          const std::shared_ptr<rmw_request_id_t> request_header,
          const std::shared_ptr<antobot_devices_msgs::srv::ProgressUpdate::Request> req, 
          const std::shared_ptr<antobot_devices_msgs::srv::ProgressUpdate::Response> res)
        {
          RCLCPP_INFO(this->get_logger(), "HMI button pressed - set hmi_auto_button_pressed True %d",req->progress_code);
          hmi_auto_button_pressed = true;
          hmi_auto_button_time = this->now();
        }

        bool checkObstacle(void)
        {
          // if obstacle is detected - will update after looking at Graph obstacle detection
          // return true
          if (footprint_received && costmap_received){ // Assuming the costmap is bigger than the footprint
            // w (x) to check - only front of the robot for 1m
            int tmp_x = int(width/2 + footprint_x/resolution);
            int tmp_check = std::min(tmp_x + int(1.0/resolution),width);
            // h (y) to check - width of the robot 
            int tmp_y_low = int(height/2 - footprint_y/resolution);
            int tmp_y_high = int(height/2 + footprint_y/resolution);
            bool obstacle_found = false;
            //ROS_INFO("%d %d %d %d", tmp_x, tmp_check, tmp_y_low, tmp_y_high); // this is in cell unit
            for(int x = tmp_x; x <tmp_check; x++){
                for(int y = tmp_y_low; y <tmp_y_high; y++){
                    //ROS_INFO("x %d y %d value %d", x, y, costmap_msg[x][y]);
                    if (costmap_msg[x][y] != 0){
                        RCLCPP_INFO(this->get_logger(), "obstacle found at x %d y %d",x, y );
                        obstacle_found = true;
                        break;
                    }
                }
                if(obstacle_found){
                    break;
                }
            }
            if(obstacle_found){
                return true;
            }
            else{
                return false;
            }
          }
          else{
              RCLCPP_INFO(this->get_logger(), "footprint and costmap not recieved - obstacle not checked");
              return false; 
          }
        }

        bool dualGPSHeadingCalibration(void)
        {
          // only if the recevied heading is up to date
          if ((this->now() - heading_received_time).seconds() < 1){ 
            // gps_yaw is dual gps heading in -pi and +pi 
            float gps_yaw_tmp = double(dual_gps_heading.data)*M_PI/180.0;
            // Adjust the angle to be within -PI to +PI
            if (gps_yaw_tmp > M_PI) {
                gps_yaw_tmp -= 2.0 * M_PI;
            }
            gps_yaw = gps_yaw_tmp;
            
            if (imu_calibration_status < 0){ // gps_end used only for the initial calibration 
                // gps_end save
                gps_end.clear();
                gps_end.push_back(utm_x);
                gps_end.push_back(utm_y); 
            }
            return true;
            
          }
          else{
            RCLCPP_INFO(this->get_logger(), "dual GPS heading value not updated");
            return false;
          }
        }
        
        // Callback functions 
        void gpsCallback(const sensor_msgs::msg::NavSatFix &msg)
        {
          // Description: GPS callback function that gets utm coordinates(from lat, long values) and rts status
          // UTMNorthing, UTMEasting, UTMZone
          GeonavTransform::NavsatConversions::LLtoUTM(msg.latitude, msg.longitude, utm_y, utm_x, utm_zone); //convert from gps to utm coordinates 

          rtk_status = msg.status.status;
          if (!gps_received){
              gps_received = true;
          }
        }

        void headingCallback(const std_msgs::msg::Float64 &msg) // dual GPS
        {
          if (!heading_received){
            heading_received = true;
          }
          dual_gps_heading = msg; // ENU 0 to 360 deg 
          heading_received_time = this->now();
        }

        void imuCallback(const sensor_msgs::msg::Imu &msg)
        {
          // Description: IMU callback function (IMU sensor raw data)
          q_imu = msg.orientation;
          imu_ang_vel_z = msg.angular_velocity.z; //value used for checking angular vel
          if (!imu_received){
              imu_received = true;
          }
        }

        void odometryCallback(const nav_msgs::msg::Odometry &msg)
        {
          // Description: EKF odometry topic callback function.
          //EKF odometry orientation is the same as calibrated imu's orientation
          q_odom = msg.pose.pose.orientation; // value to compare with gps yaw degree
          if (!odometry_received){
              odometry_received = true;
          }
        }

        void wheelOdomCallback(const nav_msgs::msg::Odometry &msg)
        {
          // Description: Wheel odometry callback function that gets linear x velocity 
          wheel_odom_v = msg.twist.twist.linear.x; // value used for checking linear vel
        } 

        void costmapCallback(const nav_msgs::msg::OccupancyGrid &msg)
        {
          // Description: costmap callback function  
          // TODO: check if the costmap is empty due to no lidar or no lidar frame
          if (!costmap_received){
            costmap_received = true;
            width = msg.info.width;
            height = msg.info.height;
            resolution = msg.info.resolution;
            costmap_msg.resize(width,std::vector<int>(height));
          }
          if (imu_calibration_status == -1){ // update costmap only during the initial calibration
              for (unsigned int x = 0; x < width; x++){
                  for (unsigned int y = 0; y < height; y++){
                      //ROS_INFO("%d",msg->data[x+ width * y]);
                      costmap_msg[x][y]=(msg.data[x+ width * y]);
                  }
              }
          }
        }

        void footprintCallback(const geometry_msgs::msg::PolygonStamped &msg)
        {
          if (!footprint_received){
            footprint_received = true;
            //Assumming footprint points are 
            footprint_x = abs(msg.polygon.points[0].x);
            footprint_y = abs(msg.polygon.points[0].y);
            //ROS_INFO("%f %f",footprint_x, footprint_y);
          }
        }

        void cmdVelCallback(const geometry_msgs::msg::Twist &msg)
        {
          robot_cmd_vel = msg;
        }
};
//}


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AntobotHeading>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;

  return(0);
}