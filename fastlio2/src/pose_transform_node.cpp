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
  if (base_config["velocity_in_carbody"]) {
    config_.velocity_in_carbody = base_config["velocity_in_carbody"].as<bool>();
  }
  if (base_config["calculate_by_average"]) {
    config_.calculate_by_average = base_config["calculate_by_average"].as<bool>();
  }
}

// 低频的位姿数据（imu系）
void PoseTransformNode::lidarFrecPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  V3D trans(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond rot(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
  MinPose T_w_imu(trans, rot);
  MinPose T_w_carbody = T_w_imu * config_.T_imu_carbody;

  V3D vel, gyro;

  // 如果存在上一帧位姿，则计算速度和角速度
  if (has_last_lidar_pose_) {
    double dt = rclcpp::Time(msg->header.stamp).seconds() - last_lidar_frec_pose_time_;
    calculateVelocityFromPoses(last_lidar_frec_pose_, T_w_carbody, dt, vel, gyro);
    if (config_.velocity_in_carbody) {
      MinPose T_carbody_w = T_w_carbody.inverse();
      vel = T_carbody_w.rot * vel;
      gyro = T_carbody_w.rot * gyro;
    }
  } else {
    // 第一帧没有历史数据，速度设为零
    vel.setZero();
    gyro.setZero();
  }

  if (!config_.calculate_by_average) {
    calculateCarVelocityAndGyroInWorld(msg, T_w_carbody.rot, vel, gyro);
  }
  nav_msgs::msg::Odometry standard_msg =
      wrapStandardPoseMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot, vel, gyro);
  lidar_frec_pose_pub_->publish(standard_msg);

#ifdef VLN_MSGS_FOUND
  vln_msgs::msg::Localization custom_msg =
      wrapCustomLocalizationMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot);
  custom_lidar_frec_pose_pub_->publish(custom_msg);
#endif

  std::stringstream ss;
  ss << "T_w_carbody in lidar frec: t: " << T_w_carbody.trans.transpose()
     << ", q: " << T_w_carbody.rot.coeffs().transpose() << ", v: " << vel.transpose() << ", w: " << gyro.transpose();
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());

  // 更新历史位姿
  last_lidar_frec_pose_ = T_w_carbody;
  has_last_lidar_pose_ = true;
  last_lidar_frec_pose_time_ = rclcpp::Time(msg->header.stamp).seconds();

  // TODO: 如果回调函数是多线程的，需要增加互斥锁
  lidar_frec_velocity_ = vel;
  lidar_frec_angular_velocity_ = gyro;
}

// 修改 imuFrecPoseCallback 函数
void PoseTransformNode::imuFrecPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // 保存当前消息作为历史消息
  nav_msgs::msg::Odometry current_pose = *msg;

  V3D trans(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond rot(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
  MinPose T_w_imu(trans, rot);
  MinPose T_w_carbody = T_w_imu * config_.T_imu_carbody;

  // imu 的速度直接用激光频率下的速度，因为imu 频率下的速度不稳定
  nav_msgs::msg::Odometry standard_msg = wrapStandardPoseMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot,
                                                             lidar_frec_velocity_, lidar_frec_angular_velocity_);
  imu_frec_pose_pub_->publish(standard_msg);

#ifdef VLN_MSGS_FOUND
  vln_msgs::msg::Localization custom_msg =
      wrapCustomLocalizationMsg(msg->header.stamp, T_w_carbody.trans, T_w_carbody.rot);
  custom_imu_frec_pose_pub_->publish(custom_msg);
#endif
}

void PoseTransformNode::calculateCarVelocityAndGyroInWorld(const nav_msgs::msg::Odometry::SharedPtr msg,
                                                           const Eigen::Quaterniond& rot_w_car, V3D& vel_result,
                                                           V3D& gyro_result) {
  V3D vel_in_imu = V3D(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  V3D gyro_in_imu = V3D(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
  V3D gyro_in_car = config_.T_carbody_imu.rot * gyro_in_imu;
  V3D vel_in_car = config_.T_carbody_imu.rot * (vel_in_imu + SkewSymmetric(gyro_in_imu) * config_.T_imu_carbody.trans);

  V3D gyro_in_world = rot_w_car * gyro_in_car;
  V3D vel_in_world = rot_w_car * vel_in_car;

  if (config_.velocity_in_carbody) {
    vel_result = vel_in_car;
    gyro_result = gyro_in_car;
  } else {
    vel_result = vel_in_world;
    gyro_result = gyro_in_world;
  }
}

// 新增函数：从两个位姿和时间差计算速度和角速度
void PoseTransformNode::calculateVelocityFromPoses(const MinPose& pose1, const MinPose& pose2, double dt, V3D& velocity,
                                                   V3D& angular_velocity) {
  if (dt <= 0.0 || dt > 1.0) {
    velocity.setZero();
    angular_velocity.setZero();
    return;
  }

  // 线速度计算
  velocity = (pose2.trans - pose1.trans) / dt;

  // 计算相对旋转
  Eigen::Quaterniond diff_quat = pose2.rot.inverse() * pose1.rot.inverse();

  // 转换为轴角表示并计算角速度
  Eigen::AngleAxisd angle_axis(diff_quat);
  angular_velocity = angle_axis.axis() * angle_axis.angle() / dt;
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
                                                               const V3D& trans, const Eigen::Quaterniond& rot,
                                                               const V3D& vel, const V3D& gyro) {
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

  msg.twist.twist.linear.x = vel.x();
  msg.twist.twist.linear.y = vel.y();
  msg.twist.twist.linear.z = vel.z();
  msg.twist.twist.angular.x = gyro.x();
  msg.twist.twist.angular.y = gyro.y();
  msg.twist.twist.angular.z = gyro.z();
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