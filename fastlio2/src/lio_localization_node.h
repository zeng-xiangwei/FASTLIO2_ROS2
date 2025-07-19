#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "icp_localizer.h"
#include "lio_node.h"

struct LocalizationConfig {
  std::string global_map_file;
  ICPConfig icp_config;
};

class LIOLocalizationNode : public LIONode {
 public:
  explicit LIOLocalizationNode(const std::string& node_name);

  virtual void initRos() override;
  virtual bool ready() override;

  virtual void loadParameters() override;

  virtual void saveLatestLidarPose() override;

  void relocalization();

  void clearDataBuffer();

  void initPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

 private:
  void publishGlobalMap(CloudType::Ptr cloud);

  // 返回值表示是否得到的初始位姿
  bool getInitPose(Eigen::Vector3d& init_pos, Eigen::Quaterniond& init_rot);
  // 获取当前激光点云
  CloudType::Ptr getInitLidarCloud();

  // 读取保存的位姿，用于重定位 time x y z qx qy qz qw
  void readSavedPose(const std::string& file_path);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_relocalization_pose_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_map_pub;

  std::shared_ptr<Pose> m_relocalization_init_pose;

  LocalizationConfig m_localization_config;
  std::shared_ptr<ICPLocalizer> m_icp_localizer;
  bool m_relocalize_success = false;

  // 上一次程序关闭后保存的定位位姿，用于重定位
  bool m_try_saved_pose = true;
  Eigen::Vector3d m_saved_pos;
  Eigen::Quaterniond m_saved_rot;
  std::string m_saved_pose_file_path;
  // 记录最新的位姿，用于在位姿变化时保存到文件
  Eigen::Vector3d m_latest_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond m_latest_rot = Eigen::Quaterniond::Identity();
};