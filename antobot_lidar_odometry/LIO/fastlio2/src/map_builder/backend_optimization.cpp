#include "backend_optimization.h"
#include "loop_closure.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace fastlio2 {

BackendOptimization::BackendOptimization(std::shared_ptr<rclcpp::Node> node, const BackendConfig& config)
    : node_(node), config_(config), running_(false), last_optimized_id_(-1), first_pose_added_(false) {

    // Initialize ISAM2
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = config_.relinearize_threshold;
    isam_params.relinearizeSkip = config_.relinearize_skip;
    isam2_ = std::make_unique<gtsam::ISAM2>(isam_params);

    // Initialize publishers
    if (config_.publish_optimized_path) {
        pub_optimized_path_ = node_->create_publisher<nav_msgs::msg::Path>(
            config_.optimized_path_topic, 10);
    }

    // Initialize TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

BackendOptimization::~BackendOptimization() {
    stop();
}

void BackendOptimization::addOdometryFactor(int id, const gtsam::Pose3& pose, double timestamp) {
    std::lock_guard<std::mutex> lock(graph_mutex_);

    gtsam::Symbol symbol('x', id);

    if (!first_pose_added_) {
        // Add prior factor for the first pose
        auto prior_noise = createPriorNoise();
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(symbol, pose, prior_noise));
        initial_estimate_.insert(symbol, pose);
        first_pose_added_ = true;

        RCLCPP_INFO(node_->get_logger(), "Added prior factor for pose %d", id);
    } else {
        // Add between factor
        gtsam::Symbol prev_symbol('x', id - 1);
        if (pose_map_.find(id - 1) != pose_map_.end()) {
            gtsam::Pose3 prev_pose = pose_map_[id - 1];
            gtsam::Pose3 relative_pose = prev_pose.between(pose);

            auto odom_noise = createOdometryNoise();
            graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_symbol, symbol, relative_pose, odom_noise));
            initial_estimate_.insert(symbol, pose);

            RCLCPP_DEBUG(node_->get_logger(), "Added odometry factor between pose %d and %d", id - 1, id);
        }
    }

    pose_map_[id] = pose;

    // Store timestamp
    if (static_cast<int>(pose_timestamps_.size()) <= id) {
        pose_timestamps_.resize(id + 1);
    }
    pose_timestamps_[id] = timestamp;
}

void BackendOptimization::addLoopConstraint(int current_id, int matched_id, const gtsam::Pose3& relative_pose,
                                           gtsam::noiseModel::Diagonal::shared_ptr noise_model) {
    std::lock_guard<std::mutex> lock(loop_mutex_);

    loop_constraint_queue_.emplace(current_id, matched_id, relative_pose, noise_model);

    RCLCPP_INFO(node_->get_logger(), "Added loop constraint between pose %d and %d", current_id, matched_id);
}

void BackendOptimization::start() {
    if (!config_.enable || running_) return;

    running_ = true;
    optimization_thread_ = std::thread(&BackendOptimization::optimizationThread, this);

    RCLCPP_INFO(node_->get_logger(), "Backend optimization started");
}

void BackendOptimization::stop() {
    running_ = false;
    if (optimization_thread_.joinable()) {
        optimization_thread_.join();
    }

    RCLCPP_INFO(node_->get_logger(), "Backend optimization stopped");
}

void BackendOptimization::optimizationThread() {
    rclcpp::Rate rate(config_.optimization_frequency);

    while (running_ && rclcpp::ok()) {
        processLoopConstraints();
        performOptimization();
        updateOptimizedPoses();
        publishOptimizedPath();
        broadcastTransform();

        rate.sleep();
    }
}

void BackendOptimization::processLoopConstraints() {
    std::lock_guard<std::mutex> loop_lock(loop_mutex_);
    std::lock_guard<std::mutex> graph_lock(graph_mutex_);

    while (!loop_constraint_queue_.empty()) {
        const auto& constraint = loop_constraint_queue_.front();

        gtsam::Symbol current_symbol('x', constraint.current_id);
        gtsam::Symbol matched_symbol('x', constraint.matched_id);

        // Add loop closure factor
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            current_symbol, matched_symbol, constraint.relative_pose, constraint.noise_model));

        RCLCPP_INFO(node_->get_logger(), "Processed loop constraint between pose %d and %d",
                   constraint.current_id, constraint.matched_id);

        loop_constraint_queue_.pop();
    }
}

void BackendOptimization::performOptimization() {
    std::lock_guard<std::mutex> lock(graph_mutex_);

    if (graph_.empty()) return;

    try {
        // Update ISAM2
        isam2_->update(graph_, initial_estimate_);
        isam2_->update();

        // Get optimized estimates
        optimized_estimates_ = isam2_->calculateEstimate();

        // Clear graph and initial estimate for next iteration
        graph_.resize(0);
        initial_estimate_.clear();

        RCLCPP_DEBUG(node_->get_logger(), "Performed backend optimization with %zu poses",
                    optimized_estimates_.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Backend optimization failed: %s", e.what());
    }
}

void BackendOptimization::updateOptimizedPoses() {
    if (optimized_estimates_.empty()) return;

    // Update pose map with optimized poses
    for (const auto& key_value : optimized_estimates_) {
        gtsam::Symbol symbol(key_value.key);
        if (symbol.chr() == 'x') {
            int id = symbol.index();
            gtsam::Pose3 optimized_pose = optimized_estimates_.at<gtsam::Pose3>(key_value.key);

            pose_map_[id] = optimized_pose;

            // Call pose update callback if set
            if (pose_update_callback_) {
                pose_update_callback_(id, optimized_pose);
            }

            last_optimized_id_ = std::max(last_optimized_id_, id);
        }
    }
}

void BackendOptimization::publishOptimizedPath() {
    if (!config_.publish_optimized_path || !pub_optimized_path_ || pose_map_.empty()) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = node_->now();
    path_msg.header.frame_id = config_.map_frame;

    for (const auto& pose_pair : pose_map_) {
        int id = pose_pair.first;
        const gtsam::Pose3& pose = pose_pair.second;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = node_->now();
        pose_stamped.header.frame_id = config_.map_frame;

        // Convert gtsam::Pose3 to geometry_msgs::Pose
        pose_stamped.pose.position.x = pose.translation().x();
        pose_stamped.pose.position.y = pose.translation().y();
        pose_stamped.pose.position.z = pose.translation().z();

        gtsam::Quaternion quat = pose.rotation().toQuaternion();
        pose_stamped.pose.orientation.w = quat.w();
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();

        path_msg.poses.push_back(pose_stamped);
    }

    pub_optimized_path_->publish(path_msg);
}

void BackendOptimization::broadcastTransform() {
    if (pose_map_.empty()) return;

    // Get the latest optimized pose
    auto latest_it = pose_map_.rbegin();
    if (latest_it == pose_map_.rend()) return;

    const gtsam::Pose3& latest_pose = latest_it->second;

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = node_->now();
    transform_stamped.header.frame_id = config_.map_frame;
    transform_stamped.child_frame_id = config_.odom_frame;

    transform_stamped.transform.translation.x = latest_pose.translation().x();
    transform_stamped.transform.translation.y = latest_pose.translation().y();
    transform_stamped.transform.translation.z = latest_pose.translation().z();

    gtsam::Quaternion quat = latest_pose.rotation().toQuaternion();
    transform_stamped.transform.rotation.w = quat.w();
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();

    tf_broadcaster_->sendTransform(transform_stamped);
}

gtsam::Pose3 BackendOptimization::getOptimizedPose(int id) const {
    auto it = pose_map_.find(id);
    if (it != pose_map_.end()) {
        return it->second;
    }
    return gtsam::Pose3();
}

std::vector<gtsam::Pose3> BackendOptimization::getAllOptimizedPoses() const {
    std::vector<gtsam::Pose3> poses;
    for (const auto& pose_pair : pose_map_) {
        poses.push_back(pose_pair.second);
    }
    return poses;
}

bool BackendOptimization::hasOptimizedPose(int id) const {
    return pose_map_.find(id) != pose_map_.end();
}

void BackendOptimization::setPoseUpdateCallback(std::function<void(int, const gtsam::Pose3&)> callback) {
    pose_update_callback_ = callback;
}

gtsam::noiseModel::Diagonal::shared_ptr BackendOptimization::createOdometryNoise() {
    gtsam::Vector6 noise_vector;
    noise_vector << config_.odom_noise_rotation, config_.odom_noise_rotation, config_.odom_noise_rotation,
                    config_.odom_noise_translation, config_.odom_noise_translation, config_.odom_noise_translation;
    return gtsam::noiseModel::Diagonal::Variances(noise_vector);
}

gtsam::noiseModel::Diagonal::shared_ptr BackendOptimization::createPriorNoise() {
    gtsam::Vector6 noise_vector;
    noise_vector << config_.prior_noise_rotation, config_.prior_noise_rotation, config_.prior_noise_rotation,
                    config_.prior_noise_translation, config_.prior_noise_translation, config_.prior_noise_translation;
    return gtsam::noiseModel::Diagonal::Variances(noise_vector);
}

} // namespace fastlio2