#pragma once

#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <Eigen/Dense>

namespace fastlio2 {

// Forward declaration
struct LoopConstraint;

struct BackendConfig {
    bool enable = true;
    double optimization_frequency = 1.0;

    // ISAM2 parameters
    double relinearize_threshold = 0.1;
    int relinearize_skip = 1;

    // Prior factor noise
    double prior_noise_translation = 1e-6;
    double prior_noise_rotation = 1e-4;

    // Odometry factor noise
    double odom_noise_translation = 1e-6;
    double odom_noise_rotation = 1e-4;

    // Loop closure factor noise
    double loop_noise_translation = 0.5;
    double loop_noise_rotation = 0.5;

    // Path publishing
    bool publish_optimized_path = true;
    std::string optimized_path_topic = "optimized_path";
    std::string map_frame = "map";
    std::string odom_frame = "odom";
};

struct LoopConstraint {
    int current_id;
    int matched_id;
    gtsam::Pose3 relative_pose;
    gtsam::noiseModel::Diagonal::shared_ptr noise_model;

    LoopConstraint(int curr_id, int match_id, const gtsam::Pose3& pose,
                   gtsam::noiseModel::Diagonal::shared_ptr noise)
        : current_id(curr_id), matched_id(match_id), relative_pose(pose), noise_model(noise) {}
};

class BackendOptimization {
public:
    BackendOptimization(std::shared_ptr<rclcpp::Node> node, const BackendConfig& config);
    ~BackendOptimization();

    // Core functions
    void addOdometryFactor(int id, const gtsam::Pose3& pose, double timestamp);
    void addLoopConstraint(int current_id, int matched_id, const gtsam::Pose3& relative_pose,
                          gtsam::noiseModel::Diagonal::shared_ptr noise_model);

    // Optimization control
    void start();
    void stop();
    void performOptimization();

    // Getters
    gtsam::Pose3 getOptimizedPose(int id) const;
    std::vector<gtsam::Pose3> getAllOptimizedPoses() const;
    bool hasOptimizedPose(int id) const;

    // Callbacks
    void setPoseUpdateCallback(std::function<void(int, const gtsam::Pose3&)> callback);

private:
    // Core optimization functions
    void optimizationThread();
    void processLoopConstraints();
    void updateOptimizedPoses();
    void publishOptimizedPath();
    void broadcastTransform();

    // Utility functions
    gtsam::noiseModel::Diagonal::shared_ptr createOdometryNoise();
    gtsam::noiseModel::Diagonal::shared_ptr createPriorNoise();

    // Node and configuration
    std::shared_ptr<rclcpp::Node> node_;
    BackendConfig config_;

    // GTSAM components
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values optimized_estimates_;

    // Threading
    std::thread optimization_thread_;
    std::atomic<bool> running_;

    // Data storage
    std::mutex graph_mutex_;
    std::mutex loop_mutex_;
    std::queue<LoopConstraint> loop_constraint_queue_;
    std::vector<double> pose_timestamps_;
    std::map<int, gtsam::Pose3> pose_map_;

    // ROS publishers and TF
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimized_path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Callbacks
    std::function<void(int, const gtsam::Pose3&)> pose_update_callback_;

    // State tracking
    int last_optimized_id_;
    bool first_pose_added_;
};

} // namespace fastlio2