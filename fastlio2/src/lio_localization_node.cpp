#include "lio_localization_node.h"

#include <glog/logging.h>

LIOLocalizationNode::LIOLocalizationNode(const std::string& node_name) : LIONode(node_name) {}

void LIOLocalizationNode::initRos() {
  LIONode::initRos();
  m_icp_localizer = std::make_shared<ICPLocalizer>(m_localization_config.icp_config);
  m_icp_localizer->loadMap(m_localization_config.global_map_file);
  m_relocalization_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1, std::bind(&LIOLocalizationNode::initPoseCB, this, std::placeholders::_1));
  m_global_map_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", rclcpp::QoS(1).transient_local());

  LOG(INFO) << "refine map points size: " << m_icp_localizer->refineMap()->size();
  publishGlobalMap(m_icp_localizer->refineMap());

  m_builder->setLocalizationGlobalMap(m_icp_localizer->refineMap());
}

bool LIOLocalizationNode::ready() {
  if (m_relocalize_success) {
    return true;
  }

  relocalization();
  return m_relocalize_success;
}

void LIOLocalizationNode::loadParameters() {
  LOG(INFO) << "in LIOLocalizationNode loadPrama";
  LIONode::loadParameters();
  this->declare_parameter("localization_config_path", "");
  std::string config_path;
  this->get_parameter<std::string>("localization_config_path", config_path);

  YAML::Node config = YAML::LoadFile(config_path);
  if (!config) {
    RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE! %s", config_path.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
  m_localization_config.global_map_file = config["global_map_file"].as<std::string>();

  m_localization_config.icp_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
  m_localization_config.icp_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
  m_localization_config.icp_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
  m_localization_config.icp_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

  m_localization_config.icp_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
  m_localization_config.icp_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
  m_localization_config.icp_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
  m_localization_config.icp_config.refine_score_thresh = config["refine_score_thresh"].as<double>();

  m_builder_config.gravity_align_to_global_map = config["gravity_align_to_global_map"].as<bool>();

  std::vector<double> t_bi_vec = config["t_carbody_imu"].as<std::vector<double>>();
  std::vector<double> r_bi_vec = config["r_carbody_imu"].as<std::vector<double>>();
  m_localization_config.t_carbody_imu << t_bi_vec[0], t_bi_vec[1], t_bi_vec[2];
  m_localization_config.r_carbody_imu << r_bi_vec[0], r_bi_vec[1], r_bi_vec[2], r_bi_vec[3], r_bi_vec[4], r_bi_vec[5],
      r_bi_vec[6], r_bi_vec[7], r_bi_vec[8];

  m_localization_config.r_imu_carbody = m_localization_config.r_carbody_imu.transpose();
  m_localization_config.t_imu_carbody = -m_localization_config.r_imu_carbody * m_localization_config.t_carbody_imu;
}

void LIOLocalizationNode::publishGlobalMap(CloudType::Ptr cloud) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = m_node_config.world_frame;
  cloud_msg.header.stamp = this->get_clock()->now();
  m_global_map_pub->publish(cloud_msg);
}

void LIOLocalizationNode::initPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_relocalization_init_pose = std::make_shared<Pose>();
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z);
  m_relocalization_init_pose->rot = M3D(q);
  m_relocalization_init_pose->trans =
      V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void LIOLocalizationNode::relocalization() {
  Eigen::Vector3d init_pos;
  Eigen::Quaterniond init_rot;

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_relocalization_init_pose == nullptr) {
      return;
    }
    LOG(INFO) << "Get initial pose!";
    init_pos = m_relocalization_init_pose->trans;
    init_rot = Eigen::Quaterniond(m_relocalization_init_pose->rot);
    m_relocalization_init_pose = nullptr;
  }

  CloudType::Ptr lidar_data = nullptr;
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_state_data.lidar_buffer.empty()) {
      LOG(WARNING) << "No lidar data received yet, waiting...";
      LOG(WARNING) << "Waiting for initial pose....";
      return;
    }

    lidar_data = m_state_data.lidar_buffer.back().second;
    clearDataBuffer();
  }

  // 进行icp匹配
  M4F transform_global_local = M4F::Identity();
  transform_global_local.topRightCorner(3, 1) = init_pos.cast<float>();
  transform_global_local.topLeftCorner(3, 3) = init_rot.toRotationMatrix().cast<float>();
  m_icp_localizer->setInput(lidar_data);
  if (m_icp_localizer->align(transform_global_local)) {
    RCLCPP_WARN(this->get_logger(), "Initial pose successfully!");
    LOG(INFO) << "icp result: \n" << transform_global_local;

    m_relocalize_success = true;

    M4D transform_i_l = M4D::Identity();
    transform_i_l.topLeftCorner(3, 3) = m_builder_config.r_il;
    transform_i_l.topRightCorner(3, 1) = m_builder_config.t_il;

    // 由于 float 转 double 存在精度丢失，这里先转为四元数，在转为矩阵
    M3F mat_wl_float = transform_global_local.topLeftCorner(3, 3);
    Eigen::Quaternionf q_wl_float(mat_wl_float);
    Eigen::Quaterniond q_wl(q_wl_float);
    q_wl.normalize();
    M4D transform_w_l = M4D::Identity();
    transform_w_l.topLeftCorner(3, 3) = q_wl.toRotationMatrix();
    transform_w_l.topRightCorner(3, 1) = transform_global_local.topRightCorner(3, 1).cast<double>();

    M4D transform_w_i = transform_w_l * transform_i_l.inverse();

    State& init_state = m_kf->x();
    init_state.r_wi = transform_w_i.topLeftCorner(3, 3);
    init_state.t_wi = transform_w_i.topRightCorner(3, 1);
    LOG(INFO) << "state after relocalization: \n" << init_state;
    return;
  }
  RCLCPP_WARN(this->get_logger(), "Waiting for initial pose....");
}

void LIOLocalizationNode::clearDataBuffer() {
  m_state_data.lidar_pushed = false;
  m_state_data.lidar_buffer.clear();
  m_state_data.imu_buffer.clear();
  m_state_data.path.poses.clear();
  m_state_data.last_imu_time = -1.0;
  m_state_data.last_lidar_time = -1.0;
}

void LIOLocalizationNode::transformToCarbody(const State& input, V3D& trans, M3D& rot, V3D& vel) {
  // T_w_car = T_w_imu * T_imu_car.inverse()
  rot = input.r_wi * m_localization_config.r_imu_carbody;
  trans = input.r_wi * m_localization_config.t_imu_carbody + input.t_wi;
  vel = rot.transpose() * input.v;
}