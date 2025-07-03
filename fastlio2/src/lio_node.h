#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>
#include <condition_variable>
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
#include "tf2_ros/transform_broadcaster.h"
#include "utils.h"

using namespace std::chrono_literals;
struct NodeConfig {
  std::string imu_topic = "/livox/imu";
  std::string lidar_topic = "/livox/lidar";
  std::string body_frame = "body";
  std::string world_frame = "lidar";
  bool print_time_cost = false;
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

  virtual void transformToCarbody(const State& input, V3D& trans, M3D& rot, V3D& vel);

  void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

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

 protected:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_imu_frec_odom_pub;

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
};