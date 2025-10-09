

// Copyright (c) 2025, ANTOBOT LTD.
// All rights reserved.

// Description: This is for keyframe manger and storage, which have support loop closure base distance;
// use icp methods to verify loop closure, creat loop constrains .

// Contacts: yu.chen@nicecart.ai
// Created by yu on 2025/10/9.

#include "loop_closure.h"
#include <pcl/common/transforms.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace fastlio2 {

LoopClosure::LoopClosure(std::shared_ptr<rclcpp::Node> node, const LoopClosureConfig& config)
    : node_(node), config_(config), running_(false) {

    kdtree_history_poses_ = std::make_shared<pcl::KdTreeFLANN<PointType>>();
    history_poses_ = std::make_shared<pcl::PointCloud<PointType>>();

    downsample_filter_.setLeafSize(0.4, 0.4, 0.4);

    // Initialize publishers
    pub_history_keyframes_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "loop_closure/history_keyframes", 10);
    pub_icp_keyframes_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "loop_closure/icp_keyframes", 10);
    pub_loop_constraint_edge_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "loop_closure/constraint_edges", 10);
}

LoopClosure::~LoopClosure() {
    stop();
}

void LoopClosure::addKeyFrame(int id, double timestamp, const Eigen::Matrix4d& pose,
                             pcl::PointCloud<PointType>::Ptr cloud) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);

    auto keyframe = std::make_shared<KeyFrame>(id, timestamp, pose, cloud);
    keyframes_.push_back(keyframe);

    // Update history poses for KD-tree search
    PointType pose_point;
    pose_point.x = pose(0, 3);
    pose_point.y = pose(1, 3);
    pose_point.z = pose(2, 3);
    pose_point.intensity = id;
    history_poses_->push_back(pose_point);
}

void LoopClosure::setLoopClosureCallback(
    std::function<void(int, int, const gtsam::Pose3&, gtsam::noiseModel::Diagonal::shared_ptr)> callback) {
    loop_callback_ = callback;
}

void LoopClosure::start() {
    if (!config_.enable || running_) return;

    running_ = true;
    loop_thread_ = std::thread(&LoopClosure::loopClosureThread, this);
}

void LoopClosure::stop() {
    running_ = false;
    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
}

void LoopClosure::loopClosureThread() {
    rclcpp::Rate rate(config_.frequency);

    while (running_ && rclcpp::ok()) {
        int current_id, matched_id;
        gtsam::Pose3 relative_pose;
        gtsam::noiseModel::Diagonal::shared_ptr noise_model;

        if (detectLoop(current_id, matched_id, relative_pose, noise_model)) {
            if (loop_callback_) {
                loop_callback_(current_id, matched_id, relative_pose, noise_model);
            }

            // Add to loop container
            loop_index_container_[current_id] = matched_id;

            RCLCPP_INFO(node_->get_logger(),
                       "Loop closure detected between keyframe %d and %d",
                       current_id, matched_id);
        }

        visualizeLoopClosure();
        rate.sleep();
    }
}

bool LoopClosure::detectLoop(int& current_id, int& matched_id, gtsam::Pose3& relative_pose,
                            gtsam::noiseModel::Diagonal::shared_ptr& noise_model) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);

    if (keyframes_.empty()) return false;

    int latest_id, closest_id;
    if (!detectLoopClosureDistance(&latest_id, &closest_id)) {
        return false;
    }

    // Extract clouds
    pcl::PointCloud<PointType>::Ptr current_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr history_cloud(new pcl::PointCloud<PointType>());

    // Get current keyframe cloud
    *current_cloud = *(keyframes_[latest_id]->cloud);

    // Get history keyframes cloud
    findNearKeyframes(history_cloud, closest_id, config_.history_search_num);

    if (current_cloud->size() < 300 || history_cloud->size() < 1000) {
        return false;
    }

    // Publish history cloud for visualization
    if (pub_history_keyframes_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*history_cloud, cloud_msg);
        cloud_msg.header.stamp = node_->now();
        cloud_msg.header.frame_id = "map";
        pub_history_keyframes_->publish(cloud_msg);
    }

    // ICP alignment
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_distance);
    icp.setMaximumIterations(config_.icp_max_iterations);
    icp.setTransformationEpsilon(config_.icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(config_.icp_euclidean_fitness_epsilon);
    icp.setRANSACIterations(0);

    icp.setInputSource(current_cloud);
    icp.setInputTarget(history_cloud);

    pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
    icp.align(*aligned_cloud);

    if (!icp.hasConverged() || icp.getFitnessScore() > config_.fitness_score_threshold) {
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Loop ICP Check Pass! Fitness score: %f", icp.getFitnessScore());

    // Publish aligned cloud for visualization
    if (pub_icp_keyframes_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*aligned_cloud, cloud_msg);
        cloud_msg.header.stamp = node_->now();
        cloud_msg.header.frame_id = "map";
        pub_icp_keyframes_->publish(cloud_msg);
    }

    // Calculate relative pose
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    Eigen::Matrix4d transformation_d = transformation.cast<double>();

    Eigen::Matrix4d current_pose = keyframes_[latest_id]->pose;
    Eigen::Matrix4d corrected_pose = transformation_d * current_pose;
    Eigen::Matrix4d history_pose = keyframes_[closest_id]->pose;

    gtsam::Pose3 pose_from = eigenToGtsam(corrected_pose);
    gtsam::Pose3 pose_to = eigenToGtsam(history_pose);

    relative_pose = pose_from.between(pose_to);

    // Create noise model based on fitness score
    double noise_score = icp.getFitnessScore();
    gtsam::Vector6 noise_vector;
    noise_vector << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;
    noise_model = gtsam::noiseModel::Diagonal::Variances(noise_vector);

    current_id = latest_id;
    matched_id = closest_id;

    return true;
}

bool LoopClosure::detectLoopClosureDistance(int* latest_id, int* closest_id) {
    if (keyframes_.empty()) return false;

    int loop_key_current = keyframes_.size() - 1;
    int loop_key_previous = -1;

    // Check if loop constraint already exists
    auto it = loop_index_container_.find(loop_key_current);
    if (it != loop_index_container_.end()) {
        return false;
    }

    // Update KD-tree
    kdtree_history_poses_->setInputCloud(history_poses_);

    // Search for nearby poses
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dis;

    PointType search_point = history_poses_->points.back();
    kdtree_history_poses_->radiusSearch(search_point, config_.history_search_radius,
                                       point_search_ind, point_search_sq_dis, 0);

    double current_time = keyframes_[loop_key_current]->timestamp;

    for (int i = 0; i < static_cast<int>(point_search_ind.size()); ++i) {
        int id = point_search_ind[i];
        if (id >= static_cast<int>(keyframes_.size())) continue;

        double time_diff = std::abs(keyframes_[id]->timestamp - current_time);
        if (time_diff > config_.history_search_time_diff) {
            loop_key_previous = id;
            break;
        }
    }

    if (loop_key_previous == -1 || loop_key_current == loop_key_previous) {
        return false;
    }

    *latest_id = loop_key_current;
    *closest_id = loop_key_previous;

    return true;
}

void LoopClosure::findNearKeyframes(pcl::PointCloud<PointType>::Ptr& near_keyframes,
                                   int key_id, int search_num) {
    near_keyframes->clear();

    int cloud_size = keyframes_.size();
    for (int i = -search_num; i <= search_num; ++i) {
        int key_near = key_id + i;
        if (key_near < 0 || key_near >= cloud_size) continue;

        // Transform point cloud to world frame
        pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*(keyframes_[key_near]->cloud), *transformed_cloud,
                                keyframes_[key_near]->pose.cast<float>());

        *near_keyframes += *transformed_cloud;
    }

    if (near_keyframes->empty()) return;

    // Downsample
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
    downsample_filter_.setInputCloud(near_keyframes);
    downsample_filter_.filter(*downsampled_cloud);
    *near_keyframes = *downsampled_cloud;
}

void LoopClosure::visualizeLoopClosure() {
    if (loop_index_container_.empty()) return;

    visualization_msgs::msg::MarkerArray marker_array;

    // Loop nodes
    visualization_msgs::msg::Marker marker_node;
    marker_node.header.frame_id = "map";
    marker_node.header.stamp = node_->now();
    marker_node.action = visualization_msgs::msg::Marker::ADD;
    marker_node.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_node.ns = "loop_nodes";
    marker_node.id = 0;
    marker_node.pose.orientation.w = 1.0;
    marker_node.scale.x = 0.3;
    marker_node.scale.y = 0.3;
    marker_node.scale.z = 0.3;
    marker_node.color.r = 0.0;
    marker_node.color.g = 0.8;
    marker_node.color.b = 1.0;
    marker_node.color.a = 1.0;

    // Loop edges
    visualization_msgs::msg::Marker marker_edge;
    marker_edge.header.frame_id = "map";
    marker_edge.header.stamp = node_->now();
    marker_edge.action = visualization_msgs::msg::Marker::ADD;
    marker_edge.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_edge.ns = "loop_edges";
    marker_edge.id = 1;
    marker_edge.pose.orientation.w = 1.0;
    marker_edge.scale.x = 0.1;
    marker_edge.color.r = 0.9;
    marker_edge.color.g = 0.9;
    marker_edge.color.b = 0.0;
    marker_edge.color.a = 1.0;

    std::lock_guard<std::mutex> lock(keyframes_mutex_);

    for (const auto& loop_pair : loop_index_container_) {
        int key_current = loop_pair.first;
        int key_previous = loop_pair.second;

        if (key_current >= static_cast<int>(keyframes_.size()) ||
            key_previous >= static_cast<int>(keyframes_.size())) continue;

        geometry_msgs::msg::Point p1, p2;

        p1.x = keyframes_[key_current]->pose(0, 3);
        p1.y = keyframes_[key_current]->pose(1, 3);
        p1.z = keyframes_[key_current]->pose(2, 3);

        p2.x = keyframes_[key_previous]->pose(0, 3);
        p2.y = keyframes_[key_previous]->pose(1, 3);
        p2.z = keyframes_[key_previous]->pose(2, 3);

        marker_node.points.push_back(p1);
        marker_node.points.push_back(p2);
        marker_edge.points.push_back(p1);
        marker_edge.points.push_back(p2);
    }

    marker_array.markers.push_back(marker_node);
    marker_array.markers.push_back(marker_edge);
    pub_loop_constraint_edge_->publish(marker_array);
}

gtsam::Pose3 LoopClosure::eigenToGtsam(const Eigen::Matrix4d& pose) {
    Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

    return gtsam::Pose3(gtsam::Rot3(rotation), gtsam::Point3(translation));
}

Eigen::Matrix4d LoopClosure::gtsamToEigen(const gtsam::Pose3& pose) {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = pose.rotation().matrix();
    result.block<3, 1>(0, 3) = pose.translation();
    return result;
}

} // namespace fastlio2