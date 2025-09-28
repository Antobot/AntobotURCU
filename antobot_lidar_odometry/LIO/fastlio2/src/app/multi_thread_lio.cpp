

// Copyright (c) 2025, ANTOBOT LTD.
// All rights reserved.


// Description: This is the ROS2 interface for fastlio2, responsible for subscribing to laser point cloud data and IMU data,
//              publishing laser odometry and odometry information transformed to the base frame,
//              while also publishing related TF and visualization data.
// Subscribes to: lidar topic (/livox/lidar)
//                imu topic (/livox/imu)
// Publishes:  laser_head/odom base/odom
// Contacts: yu.chen@nicecart.ai



#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <filesystem>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// TF2 headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// PCL and other includes
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include "../utils.h"
#include "../map_builder/commons.h"
#include "../map_builder/map_builder.h"

#include <fstream>
#include <iomanip>

std::ofstream tum_file;
std::ofstream tum_file1;

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string wheel_odometry_topic = "/antobot_robot/odom";
    std::string body_frame = "lidar";
    std::string world_frame = "laser_link_front";
    bool print_time_cost = true;
    bool turn_over_mid360 = true;
    bool out_put_base_odom = true;
    bool out_put_base_odom_to_tum = true;
    bool out_put_wheel_odom_to_tum = true;

    double lidar2base_x = -0.685;
    double lidar2base_y = 0.0;
    double lidar2base_z = 0.0;
};

/**
 * @brief 处理结果结构体
 * 用于从处理线程向发布线程传递处理结果
 */
struct ProcessResult
{
    SyncPackage package;
    std::shared_ptr<IESKF> kf;
    CloudType::Ptr body_cloud;
    CloudType::Ptr world_cloud;
    BuilderStatus status;
    double processing_time_ms;

    ProcessResult() = default;
    ProcessResult(const ProcessResult& other) = delete;
    ProcessResult& operator=(const ProcessResult& other) = delete;
    ProcessResult(ProcessResult&& other) = default;
    ProcessResult& operator=(ProcessResult&& other) = default;
};

/**
 * @brief 线程安全的队列模板类
 * 用于在不同线程之间安全地传递数据
 */
template<typename T>
class ThreadSafeQueue
{
public:
    ThreadSafeQueue() = default;

    /**
     * @brief 向队列中添加元素
     * @param item 要添加的元素
     */
    void push(T&& item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(item));
        condition_.notify_one();
    }

    /**
     * @brief 从队列中取出元素（阻塞版本）
     * @return T 队列中的元素
     */
    T pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return !queue_.empty() || shutdown_; });

        if (shutdown_ && queue_.empty()) {
            return T{};
        }

        T result = std::move(queue_.front());
        queue_.pop();
        return result;
    }

    /**
     * @brief 尝试从队列中取出元素（非阻塞版本）
     * @param item 输出参数，存储取出的元素
     * @return bool 是否成功取出元素
     */
    bool tryPop(T& item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    /**
     * @brief 获取队列大小
     * @return size_t 队列中元素的数量
     */
    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /**
     * @brief 检查队列是否为空
     * @return bool 队列是否为空
     */
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /**
     * @brief 关闭队列，唤醒所有等待的线程
     */
    void shutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        condition_.notify_all();
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable condition_;
    bool shutdown_ = false;
};

struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;

    // 新增：用于线程同步的原子变量
    std::atomic<bool> new_data_available{false};
};

class LIONode : public rclcpp::Node
{
public:
    /**
     * @brief LIONode构造函数 - 初始化多线程激光雷达惯性里程计节点
     */
    LIONode() : rclcpp::Node("antobot_lidar_odom"), shutdown_requested_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Multi-threaded LIO Node Started");
        loadParameters();

        // 订阅IMU数据
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, 100,
            std::bind(&LIONode::imuCB, this, std::placeholders::_1));

        // 订阅激光雷达数据
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            m_node_config.lidar_topic, 10,
            std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        // 订阅轮式里程计数据
        m_wheel_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            m_node_config.wheel_odometry_topic, 10,
            std::bind(&LIONode::wheelOdomCB, this, std::placeholders::_1));

        // 发布者初始化
        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("fast_lio_path", 10);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("fast_lio", 10);
        m_base_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("Odometry", 10);

        // TF broadcaster
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_odom2base_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        // 初始化核心算法组件
        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);

        // 启动处理线程
        processing_thread_ = std::thread(&LIONode::processingThreadFunction, this);

        // 启动发布线程
        publishing_thread_ = std::thread(&LIONode::publishingThreadFunction, this);

        // 使用较高频率的定时器来检查新数据
        m_timer = this->create_wall_timer(
            10ms, std::bind(&LIONode::timerCB, this));

        RCLCPP_INFO(this->get_logger(), "Multi-threaded LIO Node initialization completed");
    }

    /**
     * @brief 析构函数，确保线程安全退出
     */
    ~LIONode()
    {
        shutdown();
    }

private:
    // 线程相关成员变量
    std::thread processing_thread_;
    std::thread publishing_thread_;
    std::atomic<bool> shutdown_requested_;

    // 线程安全队列
    ThreadSafeQueue<SyncPackage> processing_queue_;
    ThreadSafeQueue<std::unique_ptr<ProcessResult>> result_queue_;

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_wheel_odom_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_base_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_odom2base_tf_broadcaster;

    nav_msgs::msg::Odometry base_odom;
    nav_msgs::msg::Odometry base_odom_last;

public:
    /**
     * @brief 安全关闭所有线程
     */
    void shutdown()
    {
        if (shutdown_requested_.load()) {
            return; // 已经关闭过了
        }

        RCLCPP_INFO(this->get_logger(), "Shutting down LIO Node threads...");
        shutdown_requested_.store(true);

        // 关闭队列，唤醒所有等待的线程
        processing_queue_.shutdown();
        result_queue_.shutdown();

        // 等待线程结束
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        if (publishing_thread_.joinable()) {
            publishing_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "All threads shut down successfully");
    }

    void loadParameters()
    {
        // 声明参数
        this->declare_parameter<std::string>("config_path", "");

        std::string config_path;
        this->get_parameter("config_path", config_path);

        if (config_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Config path not provided, using default values");
            return;
        }

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        m_node_config.turn_over_mid360 = config["turn_over_mid360"].as<bool>();
        m_node_config.out_put_base_odom = config["out_put_base_odom"].as<bool>();
        m_node_config.out_put_base_odom_to_tum = config["out_put_base_odom_to_tum"].as<bool>();
        m_node_config.out_put_wheel_odom_to_tum = config["out_put_wheel_odom_to_tum"].as<bool>();
        m_node_config.lidar2base_x = config["lidar2base_x"].as<double>();
        m_node_config.lidar2base_y = config["lidar2base_y"].as<double>();
        m_node_config.lidar2base_z = config["lidar2base_z"].as<double>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);

        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }

        m_state_data.imu_buffer.emplace_back(
            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
            V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            timestamp);
        m_state_data.last_imu_time = timestamp;

        // 标记有新数据可用
        m_state_data.new_data_available.store(true);
    }

    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num,
                                                m_builder_config.lidar_min_range,
                                                m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);

        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }

        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;

        // 标记有新数据可用
        m_state_data.new_data_available.store(true);
    }

    void wheelOdomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        double tx = msg->pose.pose.position.x;
        double ty = msg->pose.pose.position.y;
        double tz = msg->pose.pose.position.z;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        if (m_node_config.out_put_wheel_odom_to_tum)
        {
            tum_file1 << std::fixed << std::setprecision(6) << timestamp << " "
                      << tx << " " << ty << " " << tz << " "
                      << qx << " " << qy << " " << qz << " " << qw << std::endl;
        }
    }

    /**
     * @brief 轻量级定时器回调函数
     * 只负责检查是否有新数据并触发处理，避免阻塞
     */
    void timerCB()
    {
        // 检查是否有新数据需要处理
        if (!m_state_data.new_data_available.load()) {
            return;
        }

        // 尝试同步数据包
        if (syncPackage()) {
            // 将数据包发送到处理队列
            processing_queue_.push(std::move(m_package));
            m_state_data.new_data_available.store(false);
        }
    }

    /**
     * @brief 处理线程函数
     * 专门负责耗时的数据处理工作
     */
    void processingThreadFunction()
    {
        RCLCPP_INFO(this->get_logger(), "Processing thread started");

        while (!shutdown_requested_.load()) {
            // 从队列中获取待处理的数据包
            SyncPackage package = processing_queue_.pop();

            if (shutdown_requested_.load()) {
                break;
            }

            // 记录处理开始时间
            auto t1 = std::chrono::high_resolution_clock::now();

            // 执行地图构建和状态估计（耗时操作）
            m_builder->process(package);

            // 记录处理结束时间
            auto t2 = std::chrono::high_resolution_clock::now();
            double processing_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

            if (m_node_config.print_time_cost) {
                RCLCPP_WARN(this->get_logger(), "Processing time: %.2f ms", processing_time);
            }

            // 只有在建图状态下才准备发布数据
            if (m_builder->status() != BuilderStatus::MAPPING) {
                continue;
            }

            // 准备处理结果
            auto result = std::make_unique<ProcessResult>();
            result->package = std::move(package);
            result->kf = m_kf;
            result->status = m_builder->status();
            result->processing_time_ms = processing_time;

            // 转换点云到机体坐标系
            result->body_cloud = m_builder->lidar_processor()->transformCloud(
                result->package.cloud, m_kf->x().r_il, m_kf->x().t_il);

            // 转换点云到世界坐标系
            result->world_cloud = m_builder->lidar_processor()->transformCloud(
                result->package.cloud,
                m_builder->lidar_processor()->r_wl(),
                m_builder->lidar_processor()->t_wl());

            // 将结果发送到发布队列
            result_queue_.push(std::move(result));
        }

        RCLCPP_INFO(this->get_logger(), "Processing thread stopped");
    }

    /**
     * @brief 发布线程函数
     * 专门负责发布处理结果
     */
    void publishingThreadFunction()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing thread started");

        while (!shutdown_requested_.load()) {
            // 从结果队列中获取处理结果
            auto result = result_queue_.pop();

            if (shutdown_requested_.load() || !result) {
                break;
            }

            // 发布所有数据
            publishResults(*result);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing thread stopped");
    }

    /**
     * @brief 发布处理结果
     * @param result 处理结果
     */
    void publishResults(const ProcessResult& result)
    {
        double timestamp = result.package.cloud_end_time;

        // 广播TF变换
        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, timestamp);

        // 发布里程计数据
        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, timestamp);

        // 发布机器人底盘坐标系的TF变换
        publishBaseOdomTF(m_odom2base_tf_broadcaster, base_odom);

        // 发布点云数据
        publishCloud(m_body_cloud_pub, result.body_cloud, m_node_config.body_frame, timestamp);
        publishCloud(m_world_cloud_pub, result.world_cloud, m_node_config.world_frame, timestamp);

        // 发布路径数据
        publishPath(m_path_pub, m_node_config.world_frame, timestamp);
    }

    bool syncPackage()
    {
        // 检查缓冲区是否为空
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;

        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(),
                     [](PointType &p1, PointType &p2) { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }

        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }

        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                      CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
                        std::string frame_id, std::string child_frame, const double &time)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;

        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();

        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        M21D P = m_kf->P();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                odom.twist.covariance[i * 6 + j] = P(i+12, j+12);
            }
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                odom.twist.covariance[i * 6 + j + 3] = 0;
            }

        if (m_node_config.turn_over_mid360)
        {
            odom.pose.pose.position.x = -odom.pose.pose.position.x;
            odom.pose.pose.position.y = -odom.pose.pose.position.y;
            odom.pose.pose.position.z = -odom.pose.pose.position.z;
            Eigen::Quaterniond q_correction(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q(m_kf->x().r_wi);
            q = q_correction * q;
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();
            vel = q_correction.toRotationMatrix() * vel;
            odom.twist.twist.linear.x = vel.x();
            odom.twist.twist.linear.y = vel.y();
            odom.twist.twist.linear.z = vel.z();

            Eigen::Matrix3d R = q_correction.toRotationMatrix();
            Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d rot_cov = Eigen::Matrix3d::Zero();

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    pos_cov(i,j) = odom.pose.covariance[i * 6 + j];
                    rot_cov(i,j) = odom.pose.covariance[(i+3) * 6 + (j + 3)];
                }
            pos_cov = R * pos_cov * R.transpose();
            rot_cov = R * rot_cov * R.transpose();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    odom.pose.covariance[i * 6 + j] = pos_cov(i,j);
                    odom.pose.covariance[(i+3) * 6 + (j + 3)] = rot_cov(i,j);
                }
        }

        odom_pub->publish(odom);

        if (m_node_config.out_put_base_odom)
        {
            convertOdomLidarToBase(odom, base_odom);
            base_odom_last = base_odom;

            double timestamp = rclcpp::Time(base_odom.header.stamp).seconds();
            double tx = base_odom.pose.pose.position.x;
            double ty = base_odom.pose.pose.position.y;
            double tz = base_odom.pose.pose.position.z;
            double qx = base_odom.pose.pose.orientation.x;
            double qy = base_odom.pose.pose.orientation.y;
            double qz = base_odom.pose.pose.orientation.z;
            double qw = base_odom.pose.pose.orientation.w;

            if (m_node_config.out_put_base_odom_to_tum)
            {
                tum_file << std::fixed << std::setprecision(6) << timestamp << " "
                        << tx << " " << ty << " " << tz << " "
                        << qx << " " << qy << " " << qz << " " << qw << std::endl;
            }
            base_odom.child_frame_id = "base_link";
            base_odom.header.frame_id = "odom";

            m_base_odom_pub->publish(base_odom);
        }
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                     std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);

        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();

        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster,
                     std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);

        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;

        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        broad_caster->sendTransform(transformStamped);
    }

    void publishBaseOdomTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster,
                           const nav_msgs::msg::Odometry& base_odom)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = base_odom.header.stamp;
        tf_msg.header.frame_id = base_odom.header.frame_id;    // "odom"
        tf_msg.child_frame_id = base_odom.child_frame_id;      // "base_link"

        tf_msg.transform.translation.x = base_odom.pose.pose.position.x;
        tf_msg.transform.translation.y = base_odom.pose.pose.position.y;
        tf_msg.transform.translation.z = base_odom.pose.pose.position.z;

        tf_msg.transform.rotation = base_odom.pose.pose.orientation;

        broad_caster->sendTransform(tf_msg);
    }

    void convertOdomLidarToBase(const nav_msgs::msg::Odometry& odom_in,
                                nav_msgs::msg::Odometry& odom_out)
    {
        odom_out = odom_in;
        odom_out.header.frame_id = "odom";
        odom_out.child_frame_id  = "base_link";
        odom_out.header.stamp = Utils::getTime(rclcpp::Time(odom_in.header.stamp).seconds());

        Eigen::Quaterniond q_w_l(
            odom_in.pose.pose.orientation.w,
            odom_in.pose.pose.orientation.x,
            odom_in.pose.pose.orientation.y,
            odom_in.pose.pose.orientation.z);

        Eigen::Vector3d t_w_l(
            odom_in.pose.pose.position.x,
            odom_in.pose.pose.position.y,
            odom_in.pose.pose.position.z);

        Eigen::Vector3d t_l_b(m_node_config.lidar2base_x,
                              m_node_config.lidar2base_y,
                              m_node_config.lidar2base_z);

        Eigen::Vector3d t_w_b = t_w_l + q_w_l * t_l_b;

        odom_out.pose.pose.position.x = t_w_b.x();
        odom_out.pose.pose.position.y = t_w_b.y();
        odom_out.pose.pose.position.z = t_w_b.z();
        odom_out.pose.pose.orientation.w = q_w_l.w();
        odom_out.pose.pose.orientation.x = q_w_l.x();
        odom_out.pose.pose.orientation.y = q_w_l.y();
        odom_out.pose.pose.orientation.z = q_w_l.z();

        double yaw = atan2(2.0 * (q_w_l.w() * q_w_l.z() + q_w_l.x() * q_w_l.y()),
                          1.0 - 2.0 * (q_w_l.y() * q_w_l.y() + q_w_l.z() * q_w_l.z()));
        RCLCPP_INFO_STREAM(this->get_logger(), " base odom yaw: " << yaw);

        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
        J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R = q_w_l.toRotationMatrix();

        Eigen::Matrix3d skew_t_l_b;
        skew_t_l_b << 0, -t_l_b.z(), t_l_b.y(),
            t_l_b.z(), 0, -t_l_b.x(),
            -t_l_b.y(), t_l_b.x(), 0;
        J.block<3, 3>(0, 3) = -R * skew_t_l_b;

        Eigen::Matrix<double, 6, 6> cov_in;
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
            {
                cov_in(i , j) = odom_in.pose.covariance[i * 6 + j];
            }
        Eigen::Matrix<double, 6 ,6> cov_out = J * cov_in * J.transpose();

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
            {
                odom_out.pose.covariance[i * 6 + j] = cov_out(i, j);
            }

        Eigen::Matrix<double, 6, 6> twist_cov_in;
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
            {
                twist_cov_in(i , j) = odom_in.twist.covariance[i * 6 + j];
            }

        Eigen::Matrix<double, 6, 6> twist_cov_out = J * twist_cov_in * J.transpose();
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
            {
                odom_out.twist.covariance[i * 6 + j] = twist_cov_out(i, j);
            }

        Eigen::Vector3d omega_l(
            odom_in.twist.twist.angular.x,
            odom_in.twist.twist.angular.y,
            odom_in.twist.twist.angular.z);

        Eigen::Vector3d v_l(
            odom_in.twist.twist.linear.x,
            odom_in.twist.twist.linear.y,
            odom_in.twist.twist.linear.z);

        Eigen::Vector3d r_w_b = q_w_l * t_l_b;
        Eigen::Vector3d v_w_b = v_l + omega_l.cross(r_w_b);

        odom_out.twist.twist.linear.x  = v_w_b.x();
        odom_out.twist.twist.linear.y  = v_w_b.y();
        odom_out.twist.twist.linear.z  = v_w_b.z();
        odom_out.twist.twist.angular.x = omega_l.x();
        odom_out.twist.twist.angular.y = omega_l.y();
        odom_out.twist.twist.angular.z = omega_l.z();
    }
};


int main(int argc, char **argv) {
    // 定义TUM格式输出文件名
    std::string filename = "odom_tum.txt";
    std::string filename1 = "wheel_odom_tum.txt";

    // 检查里程计TUM文件是否存在，决定追加还是新建
    if (std::filesystem::exists(filename)) {
        std::cout << "File " << filename << " exists, appending data..." << std::endl;
        tum_file.open(filename, std::ios::out | std::ios::app);
    } else {
        std::cout << "File " << filename << " does not exist, creating new one..." << std::endl;
        tum_file.open(filename, std::ios::out | std::ios::trunc);
    }

    // 检查轮式里程计TUM文件是否存在，决定追加还是新建
    if (std::filesystem::exists(filename1)) {
        std::cout << "File " << filename1 << " exists, appending data..." << std::endl;
        tum_file1.open(filename1, std::ios::out | std::ios::app);
    } else {
        std::cout << "File " << filename1 << " does not exist, creating new one..." << std::endl;
        tum_file1.open(filename1, std::ios::out | std::ios::trunc);
    }

    // 检查文件是否成功打开
    if (!tum_file.is_open() || !tum_file1.is_open()) {
        std::cerr << "Failed to open TUM files!" << std::endl;
        return -1;
    }

    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建LIO节点实例
    auto node = std::make_shared<LIONode>();

    // 使用MultiThreadedExecutor处理回调
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::cout << "Starting multi-threaded executor..." << std::endl;

    try {
        // 启动多线程事件循环
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main loop: %s", e.what());
    }

    // 确保节点安全关闭
    node->shutdown();

    // 关闭文件
    if (tum_file.is_open()) {
        tum_file.close();
    }
    if (tum_file1.is_open()) {
        tum_file1.close();
    }

    // 关闭ROS2
    rclcpp::shutdown();

    std::cout << "LIO Node shutdown completed" << std::endl;
    return 0;
}