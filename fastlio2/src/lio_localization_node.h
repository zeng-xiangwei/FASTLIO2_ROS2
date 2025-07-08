#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "icp_localizer.h"
#include "lio_node.h"

#ifdef VLN_MSGS_FOUND
#include <vln_msgs/msg/localization.hpp>
#endif

struct LocalizationConfig {
  std::string global_map_file;
  ICPConfig icp_config;

  // 车体与lidar系的外参
  M3D r_carbody_imu = M3D::Identity();
  V3D t_carbody_imu = V3D::Zero();
  M3D r_imu_carbody = M3D::Identity();
  V3D t_imu_carbody = V3D::Zero();
};

class LIOLocalizationNode : public LIONode {
 public:
  explicit LIOLocalizationNode(const std::string& node_name);

  virtual void initRos() override;
  virtual bool ready() override;

  virtual void loadParameters() override;
  virtual void transformToCarbody(const State& input, V3D& trans, M3D& rot, V3D& vel) override;
  virtual void publishCustomOdometryMsg(std::string frame_id, const double& time, const V3D& trans,
                                        const M3D& rot) override;

  virtual void publishCustomOdometryMsgImuFrec(std::string frame_id, const double& time, const V3D& trans,
                                               const M3D& rot) override;

  void relocalization();

  void clearDataBuffer();

  void initPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

 private:
  void publishGlobalMap(CloudType::Ptr cloud);
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_relocalization_pose_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_map_pub;

#ifdef VLN_MSGS_FOUND
  vln_msgs::msg::Localization wrapCustomLocalizationMsg(std::string frame_id, const double& time,
                                                                    const V3D& trans, const M3D& rot);
  rclcpp::Publisher<vln_msgs::msg::Localization>::SharedPtr m_custom_odom_pub;
  rclcpp::Publisher<vln_msgs::msg::Localization>::SharedPtr m_custom_imu_frec_odom_pub;
#endif

  std::shared_ptr<Pose> m_relocalization_init_pose;

  LocalizationConfig m_localization_config;
  std::shared_ptr<ICPLocalizer> m_icp_localizer;
  bool m_relocalize_success = false;
};