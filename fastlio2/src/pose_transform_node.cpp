#include "pose_transform_node.h"

#include <yaml-cpp/yaml.h>

PoseTransformNode::PoseTransformNode(const std::string& node_name) : Node(node_name) {}

void PoseTransformNode::initRos() {
  RCLCPP_INFO(this->get_logger(), "%s Started", this->get_name());
  loadParameters();

  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
  // 发布 imu->carbody  carbody->lidar 的 tf 变化
  broadCastTF(config_.imu_frame, config_.carbody_frame, config_.T_imu_carbody.trans, config_.T_imu_carbody.rot);
  broadCastTF(config_.carbody_frame, config_.lidar_frame, config_.T_carbody_lidar.trans, config_.T_carbody_lidar.rot);

  imu_frec_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "lio_imu_frec_odom", rclcpp::QoS(10),
      std::bind(&PoseTransformNode::imuFrecPoseCallback, this, std::placeholders::_1));
  lidar_frec_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "lio_odom", rclcpp::QoS(10), std::bind(&PoseTransformNode::lidarFrecPoseCallback, this, std::placeholders::_1));

  lidar_frec_pose_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/localization/lidar_frec_pose", rclcpp::QoS(10));
  imu_frec_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/imu_frec_pose", rclcpp::QoS(10));

#ifdef VLN_MSGS_FOUND
  custom_lidar_frec_pose_pub_ =
      this->create_publisher<vln_msgs::msg::Localization>("/localization/custom_lidar_frec_pose", rclcpp::QoS(10));
  custom_imu_frec_pose_pub_ =
      this->create_publisher<vln_msgs::msg::Localization>("/localization/custom_imu_frec_pose", rclcpp::QoS(10));
#endif
}
// 读取 yaml 参数
void PoseTransformNode::loadParameters() {
  this->declare_parameter("config_path", "");
  std::string base_config_path;
  this->get_parameter<std::string>("config_path", base_config_path);

  YAML::Node base_config = YAML::LoadFile(base_config_path);
  if (!base_config) {
    RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE! %s", base_config_path.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", base_config_path.c_str());

  std::vector<double> t_il_vec = base_config["t_il"].as<std::vector<double>>();
  std::vector<double> r_il_vec = base_config["r_il"].as<std::vector<double>>();
  V3D t_il;
  t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
  M3D r_il;
  r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7],
      r_il_vec[8];

  std::vector<double> t_car_lidar_vec = base_config["t_carbody_lidar"].as<std::vector<double>>();
  std::vector<double> r_car_lidar_vec = base_config["r_carbody_lidar"].as<std::vector<double>>();
  V3D t_car_lidar;
  t_car_lidar << t_car_lidar_vec[0], t_car_lidar_vec[1], t_car_lidar_vec[2];
  M3D r_car_lidar;
  r_car_lidar << r_car_lidar_vec[0], r_car_lidar_vec[1], r_car_lidar_vec[2], r_car_lidar_vec[3], r_car_lidar_vec[4],
      r_car_lidar_vec[5], r_car_lidar_vec[6], r_car_lidar_vec[7], r_car_lidar_vec[8];

  MinPose T_imu_lidar(t_il, r_il);
  MinPose T_car_lidar(t_car_lidar, r_car_lidar);
  MinPose T_car_imu = T_car_lidar * T_imu_lidar.inverse();
  config_.T_carbody_imu = T_car_imu;
  config_.T_imu_carbody = T_car_imu.inverse();
  config_.T_carbody_lidar = T_car_lidar;
  config_.T_lidar_carbody = T_car_lidar.inverse();
  config_.map_frame = base_config["world_frame"].as<std::string>();
  config_.imu_frame = base_config["body_frame"].as<std::string>();
  config_.carbody_frame = base_config["carbody_frame"].as<std::string>();
  config_.lidar_frame = base_config["lidarbody_frame"].as<std::string>();
}

// 低频的位姿数据（imu系）
void PoseTransformNode::lidarFrecPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  V3D trans(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond rot(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
  MinPose T_w_imu(trans, rot);
  MinPose T_w_carbody = T_w_imu * config_.T_imu_carbody;
  nav_msgs::msg::Odometry standard_msg = wrapStandardPoseMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot);
  lidar_frec_pose_pub_->publish(standard_msg);

#ifdef VLN_MSGS_FOUND
  vln_msgs::msg::Localization custom_msg =
      wrapCustomLocalizationMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot);
  custom_lidar_frec_pose_pub_->publish(custom_msg);
#endif
}

// 高频的位姿数据（imu系）
void PoseTransformNode::imuFrecPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  V3D trans(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond rot(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
  MinPose T_w_imu(trans, rot);
  MinPose T_w_carbody = T_w_imu * config_.T_imu_carbody;
  nav_msgs::msg::Odometry standard_msg = wrapStandardPoseMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot);
  imu_frec_pose_pub_->publish(standard_msg);

#ifdef VLN_MSGS_FOUND
  vln_msgs::msg::Localization custom_msg =
      wrapCustomLocalizationMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot);
  custom_imu_frec_pose_pub_->publish(custom_msg);
#endif
}

void PoseTransformNode::broadCastTF(std::string frame_id, std::string child_frame, const V3D& trans,
                                    const Eigen::Quaterniond& rot) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = frame_id;
  transformStamped.header.stamp = this->now();
  transformStamped.child_frame_id = child_frame;
  transformStamped.transform.translation.x = trans.x();
  transformStamped.transform.translation.y = trans.y();
  transformStamped.transform.translation.z = trans.z();
  Eigen::Quaterniond q(rot);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  static_tf_broadcaster_->sendTransform(transformStamped);
}

nav_msgs::msg::Odometry PoseTransformNode::wrapStandardPoseMsg(const builtin_interfaces::msg::Time& time,
                                                               const V3D& trans, const Eigen::Quaterniond& rot) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = time;
  msg.header.frame_id = config_.map_frame;
  msg.pose.pose.position.x = trans.x();
  msg.pose.pose.position.y = trans.y();
  msg.pose.pose.position.z = trans.z();
  msg.pose.pose.orientation.x = rot.x();
  msg.pose.pose.orientation.y = rot.y();
  msg.pose.pose.orientation.z = rot.z();
  msg.pose.pose.orientation.w = rot.w();
  return msg;
}

#ifdef VLN_MSGS_FOUND
vln_msgs::msg::Localization PoseTransformNode::wrapCustomLocalizationMsg(const builtin_interfaces::msg::Time& time,
                                                                         const V3D& trans,
                                                                         const Eigen::Quaterniond& rot) {
  vln_msgs::msg::Localization msg;
  msg.header.frame_id = config_.map_frame;
  msg.header.stamp = time;
  msg.pose.position.x = trans.x();
  msg.pose.position.y = trans.y();
  msg.pose.position.z = trans.z();
  msg.pose.orientation.x = rot.x();
  msg.pose.orientation.y = rot.y();
  msg.pose.orientation.z = rot.z();
  msg.pose.orientation.w = rot.w();

  msg.confidence = 1.0;
  msg.valid_flag = 1;
  return msg;
}
#endif