#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "map_builder/commons.h"
#ifdef VLN_MSGS_FOUND
#include <vln_msgs/msg/localization.hpp>
#endif

/**
 * @brief 接受 fast-lio 发出的 imu 系的位姿，根据特定需求进行转换
 * 1. 要求定位模块发布车体系的位姿
 * 2. 将 imu 系的位姿转换成 carbody 系的位姿
 * 3. 发布自定义的位姿消息
 */
class PoseTransformNode : public rclcpp::Node {
 public:
  struct Config {
    MinPose T_carbody_imu = MinPose(V3D::Zero(), M3D::Identity());
    MinPose T_imu_carbody = MinPose(V3D::Zero(), M3D::Identity());
    std::string map_frame = "map";
    std::string imu_frame = "body";
    std::string carbody_frame = "carbody";
  };

  PoseTransformNode(const std::string& node_name);

  void initRos();

 protected:
  // 读取 yaml 参数
  void loadParameters();
  // 低频的位姿数据（imu系）
  void lidarFrecPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // 高频的位姿数据（imu系）
  void imuFrecPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  nav_msgs::msg::Odometry wrapStandardPoseMsg(const builtin_interfaces::msg::Time& time, const V3D& trans,
                                              const Eigen::Quaterniond& rot);
  void broadCastTF(std::string frame_id, std::string child_frame, const V3D& trans, const Eigen::Quaterniond& rot);

 private:
  Config config_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_frec_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_frec_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_frec_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_frec_pose_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
#ifdef VLN_MSGS_FOUND
  vln_msgs::msg::Localization wrapCustomLocalizationMsg(const builtin_interfaces::msg::Time& time, const V3D& trans,
                                                        const Eigen::Quaterniond& rot);
  rclcpp::Publisher<vln_msgs::msg::Localization>::SharedPtr custom_lidar_frec_pose_pub_;
  rclcpp::Publisher<vln_msgs::msg::Localization>::SharedPtr custom_imu_frec_pose_pub_;
#endif
};