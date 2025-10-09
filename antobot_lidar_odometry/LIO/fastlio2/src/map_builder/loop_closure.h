//
// Created by yu on 2025/10/9.
//

#ifndef FASTLIO2_LOOP_CLOSURE_H
#define FASTLIO2_LOOP_CLOSURE_H
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "commons.h"

namespace fastlio2 {

struct LoopClosureConfig {
    bool enable = false;
    double frequency = 1.0;
    double history_search_radius = 25.0;
    double history_search_time_diff = 25.0;
    int history_search_num = 25;
    double fitness_score_threshold = 0.4;
    double icp_max_correspondence_distance = 50.0;
    int icp_max_iterations = 100;
    double icp_transformation_epsilon = 1e-6;
    double icp_euclidean_fitness_epsilon = 1e-6;
};

struct KeyFrame {
    int id;
    double timestamp;
    Eigen::Matrix4d pose;
    pcl::PointCloud<PointType>::Ptr cloud;

    KeyFrame(int id_, double timestamp_, const Eigen::Matrix4d& pose_,
             pcl::PointCloud<PointType>::Ptr cloud_)
        : id(id_), timestamp(timestamp_), pose(pose_), cloud(cloud_) {}
};

class LoopClosure {
public:
    LoopClosure(std::shared_ptr<rclcpp::Node> node, const LoopClosureConfig& config);
    ~LoopClosure();

    void addKeyFrame(int id, double timestamp, const Eigen::Matrix4d& pose,
                     pcl::PointCloud<PointType>::Ptr cloud);

    bool detectLoop(int& current_id, int& matched_id, gtsam::Pose3& relative_pose,
                    gtsam::noiseModel::Diagonal::shared_ptr& noise_model);

    void setLoopClosureCallback(std::function<void(int, int, const gtsam::Pose3&,
                                                   gtsam::noiseModel::Diagonal::shared_ptr)> callback);

    void start();
    void stop();

private:
    void loopClosureThread();
    bool detectLoopClosureDistance(int* latest_id, int* closest_id);
    void findNearKeyframes(pcl::PointCloud<PointType>::Ptr& near_keyframes,
                          int key_id, int search_num);
    void visualizeLoopClosure();

    gtsam::Pose3 eigenToGtsam(const Eigen::Matrix4d& pose);
    Eigen::Matrix4d gtsamToEigen(const gtsam::Pose3& pose);

private:
    std::shared_ptr<rclcpp::Node> node_;
    LoopClosureConfig config_;

    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    std::map<int, int> loop_index_container_;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_history_poses_;
    pcl::PointCloud<PointType>::Ptr history_poses_;

    pcl::VoxelGrid<PointType> downsample_filter_;

    std::mutex keyframes_mutex_;
    std::thread loop_thread_;
    bool running_;

    std::function<void(int, int, const gtsam::Pose3&, gtsam::noiseModel::Diagonal::shared_ptr)> loop_callback_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_history_keyframes_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_icp_keyframes_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loop_constraint_edge_;

};

} // namespace fastlio2
#endif //FASTLIO2_LOOP_CLOSURE_H