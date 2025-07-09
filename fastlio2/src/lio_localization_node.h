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

  void relocalization();

  void clearDataBuffer();

  void initPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

 private:
  void publishGlobalMap(CloudType::Ptr cloud);
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_relocalization_pose_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_map_pub;

  std::shared_ptr<Pose> m_relocalization_init_pose;

  LocalizationConfig m_localization_config;
  std::shared_ptr<ICPLocalizer> m_icp_localizer;
  bool m_relocalize_success = false;
};