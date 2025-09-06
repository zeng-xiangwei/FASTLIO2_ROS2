#pragma once
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>
// #include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>

#include "map_builder/commons.h"
#include "map_builder/imu_pose_predictor.h"
#include "map_builder/map_builder.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "utils.h"

#ifdef VLN_MSGS_FOUND
#include <vln_msgs/srv/query_cloud_points.hpp>
#endif

using namespace std::chrono_literals;

const std::string kLivoxLidarType = "livox";
const std::string kRobosenseLidarType = "robosense";

struct NodeConfig {
  std::string imu_topic = "/livox/imu";
  std::string lidar_topic = "/livox/lidar";
  std::string body_frame = "body";
  std::string world_frame = "lidar";
  std::string lidarbody_frame = "lidarbody";
  std::string arm_base_frame = "armbasebody";
  bool print_time_cost = false;
  int ros_spin_thread = 1;

  // lidar_type: livox、robosense
  std::string lidar_type = kLivoxLidarType;
  M3D imu_data_preprocess_rot = M3D::Identity();
  int n_scans = 96;

  // 过滤打到车体上的臂的点云
  Eigen::Vector3f filter_box_min = Eigen::Vector3f::Zero();
  Eigen::Vector3f filter_box_max = Eigen::Vector3f::Zero();

  // 仅用于 vla 测试
  float vla_lidar_remove_x_min = -0.2;
  float vla_lidar_remove_x_max = 0.8;
  float vla_lidar_remove_y_min = -0.7;
  float vla_lidar_remove_y_max = 0.7;
  float vla_lidar_remove_z_min = 0.0;
  float vla_lidar_remove_z_max = 1.2;
  float vla_radius = 5.0;
  float vla_transform_lookup_time = 0.1;
};
struct StateData {
  bool lidar_pushed = false;
  double last_lidar_time = -1.0;
  double last_imu_time = -1.0;
  std::deque<IMUData> imu_buffer;
  std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
  nav_msgs::msg::Path path;
  bool has_new_data = false;
};

class LIONode : public rclcpp::Node {
 public:
  explicit LIONode(const std::string& node_name);
  virtual ~LIONode();

  virtual void initRos();
  virtual bool ready();

  virtual void loadParameters();
  virtual void saveLatestLidarPose() {}

  void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
  void livoxLidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void robosenseLidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool syncPackage();

  void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud,
                    std::string frame_id, const double& time);

  void publishLiOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
                         std::string child_frame, const double& time);
  void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
                       std::string child_frame, const double& time, const V3D& trans, const M3D& rot,
                       const V3D& vel = V3D::Zero());

  void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id,
                   const double& time);

  void broadLiCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
                     std::string child_frame, const double& time);

  void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
                   std::string child_frame, const double& time, const V3D& trans, const M3D& rot);

  void timerCB();
  // 发布imu频率的位姿
  void imuFreqCB();
  void loopThread();

  const NodeConfig& getNodeConfig() const { return m_node_config; }

 protected:
  CloudType::Ptr getNearPointsForVLA(const double& time);
  CloudType::Ptr getNearPointsInNewestForVLA(const double& time, CloudType::Ptr cloud_in_body = nullptr);
  void publishVLACloud(const double& time, CloudType::Ptr cloud_in_body);
  void readParamForVLA(YAML::Node config);
  void readParamFilterBox(YAML::Node config);
  CloudType::Ptr transformCloud(CloudType::Ptr inp, const Eigen::Quaterniond& r, const V3D& t);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_livox_lidar_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_robosense_lidar_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidarbody_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_VLA_cloud_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_imu_frec_odom_pub;

#ifdef VLN_MSGS_FOUND
  rclcpp::Service<vln_msgs::srv::QueryCloudPoints>::SharedPtr m_get_near_points_srv;
  void getNearPointsCB(const std::shared_ptr<vln_msgs::srv::QueryCloudPoints::Request> request,
                       std::shared_ptr<vln_msgs::srv::QueryCloudPoints::Response> response);
#endif

  rclcpp::TimerBase::SharedPtr m_imu_freq_timer;
  StateData m_state_data;
  SyncPackage m_package;
  NodeConfig m_node_config;
  Config m_builder_config;
  std::shared_ptr<IESKF> m_kf;
  std::shared_ptr<MapBuilder> m_builder;
  std::shared_ptr<ImuPosePredictor> m_imu_pose_predictor;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  std::shared_ptr<StateWithTime> m_last_imu_frec_state;

  mutable std::mutex m_mutex;
  std::condition_variable m_condition;
  std::thread m_thread;
  bool m_finished = false;

  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  // 临时功能，提供获取附近点云的服务
  std::mutex m_vla_mutex;
  CloudType::Ptr m_vla_cloud_in_body;
  MinPose m_vla_body_pose;
  double m_vla_cloud_time = -1.0;

  // debug
  double debug_last_lidar_sensor_time_ = -1.0;
};