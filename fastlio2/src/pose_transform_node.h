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
    MinPose T_carbody_lidar = MinPose(V3D::Zero(), M3D::Identity());
    MinPose T_lidar_carbody = MinPose(V3D::Zero(), M3D::Identity());
    std::string map_frame = "map";
    std::string imu_frame = "body";
    std::string lidar_frame = "lidarbody";
    std::string carbody_frame = "carbody";

    // 线速度、角速度是否表示在车体系下，否则表示在世界系下
    bool velocity_in_carbody = false;
    // 是否使用前后两帧计算速度，true: 使用前后两帧计算速度，false: 使用 fastlio 优化出的速度
    bool calculate_by_average = false;
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
                                              const Eigen::Quaterniond& rot, const V3D& vel = V3D::Zero(),
                                              const V3D& gyro = V3D::Zero());
  void broadCastTF(std::string frame_id, std::string child_frame, const V3D& trans, const Eigen::Quaterniond& rot);

  void calculateCarVelocityAndGyroInWorld(const nav_msgs::msg::Odometry::SharedPtr msg,
                                          const Eigen::Quaterniond& rot_w_car, V3D& vel, V3D& gyro);

  void calculateVelocityFromPoses(const MinPose& pose1, const MinPose& pose2, double dt, V3D& velocity,
                                  V3D& angular_velocity);

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

  MinPose last_lidar_frec_pose_;
  MinPose last_imu_frec_pose_;
  double last_lidar_frec_pose_time_ = -1.0;
  double last_imu_frec_pose_time_ = -1.0;
  bool has_last_lidar_pose_ = false;
  bool has_last_imu_pose_ = false;

  // 记录激光频率下的速度，用来作为 imu 频率下的速度
  V3D lidar_frec_velocity_;
  V3D lidar_frec_angular_velocity_;
};