#pragma once
#include <filesystem>
#include <fstream>
#include <queue>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include "slam_interfaces/srv/save_maps.hpp"
#include "pgos/commons.h"
// #include "pgos/simple_pgo.h"
#include "pgos/pgo.h"
#include "utils/occupancy_map.h"

struct GridMapConfig {
    float grid_2d_z_min = -0.2;
    float grid_2d_z_max = 1.0;
    float grid_2d_resolution = 0.05;
    int occupancy_weight = 30;
};

struct NodeConfig {
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    PgoConfig pgo_config;
    GridMapConfig grid_map_config;
};

struct NodeState {
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time;
};

class PGONode : public rclcpp::Node {
public:
    explicit PGONode(const std::string& node_name);

    void loadParameters();

    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

    // void syncCB(const CloudWithPose::ConstSharedPtr& cloud_msg,
                // const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

    void sendBroadCastTF(builtin_interfaces::msg::Time& time);

    void publishLoopMarkers(builtin_interfaces::msg::Time& time);

    void timerCB();

    void saveMapsCB(const std::shared_ptr<slam_interfaces::srv::SaveMaps::Request> request,
                std::shared_ptr<slam_interfaces::srv::SaveMaps::Response> response);

    void initPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    // void initPoseCB(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

    bool getInitPose(Eigen::Vector3d& init_pos, Eigen::Quaterniond& init_rot);

    void publishGlobalMap(CloudType::Ptr cloud);

    // virtual ~PGONode();
private:
    NodeConfig m_node_config;
    PgoConfig m_pgo_config;
    GridMapConfig m_grid_map_config;
    NodeState m_state;
    // std::shared_ptr<SimplePGO> m_pgo;
    std::shared_ptr<PGO> m_pgo;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Service<slam_interfaces::srv::SaveMaps>::SharedPtr m_save_map_srv;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_relocalization_pose_sub;
    std::shared_ptr<Pose> m_relocalization_init_pose;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_map_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_occupancy_grid_pub;

    bool initial_state = false;

    // 用于标记是否已经从位姿文件读取过初始位姿（避免重复读取）
    bool m_pose_file_loaded = false;
    // 保存从文件读取的初始位姿
    std::shared_ptr<Pose> m_file_init_pose;
    
    std::shared_ptr<utils::OccupancyMap> m_occupancy_map;

    // 帧计数器，用于控制发布频率
    int m_frame_count = 0;
    static constexpr int kOccupancyGridPublishInterval = 5;

    // 发布栅格地图消息
    void publishOccupancyGrid();
};
