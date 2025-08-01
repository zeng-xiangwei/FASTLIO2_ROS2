#include "accumulate_lidar_cloud_for_calib.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

AccumulateLidarCloudForCalibNode::AccumulateLidarCloudForCalibNode(const std::string& name) : Node(name) {}

void AccumulateLidarCloudForCalibNode::initRos() {
  rclcpp::QoS qos = rclcpp::QoS(1);
  qos.best_effort();
  m_cloud_sub.subscribe(this, "body_cloud", qos.get_rmw_qos_profile());
  m_odom_sub.subscribe(this, "lio_odom", qos.get_rmw_qos_profile());
  m_sync = std::make_shared<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
      m_cloud_sub, m_odom_sub);
  m_sync->setAgePenalty(0.1);
  m_sync->registerCallback(
      std::bind(&AccumulateLidarCloudForCalibNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));

  // m_cloud_accumulated_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  //     "/accumulated_cloud", 1);

  // m_cloud_accumulated_timer = this->create_wall_timer(
  //     std::chrono::milliseconds(1000),
  //     std::bind(&AccumulateLidarCloudForCalibNode::timerCB, this));

  this->declare_parameter("config_path", "");
  std::string config_path;
  this->get_parameter<std::string>("config_path", config_path);

  YAML::Node config = YAML::LoadFile(config_path);
  if (!config) {
    RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

  std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
  std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
  V3D t_il;
  t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
  M3D r_il;
  r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7],
      r_il_vec[8];

  m_config.T_imu_lidar = MinPose(t_il, r_il);
  m_config.T_lidar_imu = m_config.T_imu_lidar.inverse();

  this->declare_parameter("save_path", "");
  this->get_parameter("save_path", m_config.file_path);

  std::cout << "T_imu_lidar: t: " << m_config.T_imu_lidar.trans.transpose()
            << ", q: " << m_config.T_imu_lidar.rot.coeffs().transpose() << std::endl;
}

void AccumulateLidarCloudForCalibNode::save() {
  pcl::io::savePCDFile(m_config.file_path, *m_cloud_accumulated);
  pcl::io::savePCDFile(m_config.file_path + ".pcd", *m_cloud_first);
}
void AccumulateLidarCloudForCalibNode::syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                                              const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
  V3D t_w_imu;
  Eigen::Quaterniond q_w_imu;
  t_w_imu.x() = odom_msg->pose.pose.position.x;
  t_w_imu.y() = odom_msg->pose.pose.position.y;
  t_w_imu.z() = odom_msg->pose.pose.position.z;
  q_w_imu.x() = odom_msg->pose.pose.orientation.x;
  q_w_imu.y() = odom_msg->pose.pose.orientation.y;
  q_w_imu.z() = odom_msg->pose.pose.orientation.z;
  q_w_imu.w() = odom_msg->pose.pose.orientation.w;
  MinPose T_w_imu(t_w_imu, q_w_imu);
  std::cout << "msg t: " << odom_msg->pose.pose.position.x << ", " << odom_msg->pose.pose.position.y << ", "
            << odom_msg->pose.pose.position.z << std::endl;
  std::cout << "input_odom t: " << t_w_imu.transpose() << std::endl;
  std::cout << "T_w_imu: t: " << T_w_imu.trans.transpose() << ", q: " << T_w_imu.rot.coeffs().transpose() << std::endl;
  CloudType::Ptr cloud_in_imu = std::make_shared<CloudType>();
  pcl::fromROSMsg(*cloud_msg, *cloud_in_imu);

  CloudType::Ptr cloud_in_lidar = transformCloud(cloud_in_imu, m_config.T_lidar_imu);
  MinPose T_w_l = T_w_imu * m_config.T_imu_lidar;

  if (!has_first_pose) {
    has_first_pose = true;
    m_first_T_w_lidar = T_w_l;
    m_cloud_accumulated = cloud_in_lidar;
    m_cloud_first = std::make_shared<CloudType>();
    pcl::copyPointCloud(*cloud_in_lidar, *m_cloud_first);
    return;
  }

  MinPose T_l0_li = m_first_T_w_lidar.inverse() * T_w_l;
  CloudType::Ptr cloud_in_l0 = transformCloud(cloud_in_lidar, T_l0_li);
  std::cout << "T_l0_li: t: " << T_l0_li.trans.transpose() << ", q: " << T_l0_li.rot.coeffs().transpose() << std::endl;
  *m_cloud_accumulated += *cloud_in_l0;
}

CloudType::Ptr AccumulateLidarCloudForCalibNode::transformCloud(CloudType::Ptr cloud, MinPose input_transform) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = input_transform.rot.toRotationMatrix().cast<float>();
  transform.block<3, 1>(0, 3) = input_transform.trans.cast<float>();
  CloudType::Ptr ret = std::make_shared<CloudType>();
  pcl::transformPointCloud(*cloud, *ret, transform);
  return ret;
}

void AccumulateLidarCloudForCalibNode::timerCB() {
  CloudType::Ptr ret = std::make_shared<CloudType>();
  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setLeafSize(0.1, 0.1, 0.1);
  voxel_filter.setInputCloud(m_cloud_accumulated);
  voxel_filter.filter(*ret);
  sensor_msgs::msg::PointCloud2 pub_accumulated_cloud_msg;
  pcl::toROSMsg(*ret, pub_accumulated_cloud_msg);
  pub_accumulated_cloud_msg.header.frame_id = "rslidar";
  pub_accumulated_cloud_msg.header.stamp = this->now();
  m_cloud_accumulated_pub->publish(pub_accumulated_cloud_msg);
}