#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include "map_builder/commons.h"

/**
 * @brief 接收 imu 系的点云和位姿，将点云累加到第一帧点云坐标系中，用于激光与相机的标定
 *
 */
class AccumulateLidarCloudForCalibNode : public rclcpp::Node {
 public:
  struct Config {
    MinPose T_imu_lidar;
    MinPose T_lidar_imu;
    std::string file_path;
    std::string frame_id;
  };
  AccumulateLidarCloudForCalibNode(const std::string& name);

  void initRos();
  void save();

 private:
  void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
              const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

  CloudType::Ptr transformCloud(CloudType::Ptr cloud, MinPose transform);
  void timerCB();

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>
      m_sync;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_accumulated_pub;

  rclcpp::TimerBase::SharedPtr m_cloud_accumulated_timer;

  bool has_first_pose = false;
  MinPose m_first_T_w_lidar;
  pcl::PointCloud<PointType>::Ptr m_cloud_accumulated;
  pcl::PointCloud<PointType>::Ptr m_cloud_first;

  Config m_config;
};