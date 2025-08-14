#include "lio_localization_node.h"

#include <filesystem>
#include <fstream>

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
  std::string config_path, saved_pose_file_path;
  this->get_parameter<std::string>("config_path", config_path);
  this->declare_parameter("saved_pose_file_path", "");
  this->get_parameter<std::string>("saved_pose_file_path", saved_pose_file_path);

  YAML::Node config = YAML::LoadFile(config_path);
  if (!config) {
    RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE! %s", config_path.c_str());
    return;
  }
  YAML::Node localization_config = config["localization"];
  if (!localization_config) {
    RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD localization in YAML FILE! %s", config_path.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
  m_localization_config.global_map_file = localization_config["global_map_file"].as<std::string>();

  m_localization_config.icp_config.rough_scan_resolution = localization_config["rough_scan_resolution"].as<double>();
  m_localization_config.icp_config.rough_map_resolution = localization_config["rough_map_resolution"].as<double>();
  m_localization_config.icp_config.rough_max_iteration = localization_config["rough_max_iteration"].as<int>();
  m_localization_config.icp_config.rough_score_thresh = localization_config["rough_score_thresh"].as<double>();

  m_localization_config.icp_config.refine_scan_resolution = localization_config["refine_scan_resolution"].as<double>();
  m_localization_config.icp_config.refine_map_resolution = localization_config["refine_map_resolution"].as<double>();
  m_localization_config.icp_config.refine_max_iteration = localization_config["refine_max_iteration"].as<int>();
  m_localization_config.icp_config.refine_score_thresh = localization_config["refine_score_thresh"].as<double>();

  m_builder_config.gravity_align_to_global_map = localization_config["gravity_align_to_global_map"].as<bool>();

  if (config["update_map"]) {
    m_builder_config.update_map = config["update_map"].as<bool>();
  }
  // 读取上次保留的定位结果，用于重定位
  m_saved_pose_file_path = saved_pose_file_path;
  readSavedPose(saved_pose_file_path);
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

bool LIOLocalizationNode::getInitPose(Eigen::Vector3d& init_pos, Eigen::Quaterniond& init_rot) {
  if (m_try_saved_pose) {
    LOG(INFO) << "Try to use saved pose!";
    init_pos = m_saved_pos;
    init_rot = m_saved_rot;
    return true;
  }

  if (m_relocalization_init_pose == nullptr) {
    return false;
  }
  LOG(INFO) << "Get initial pose!";
  init_pos = m_relocalization_init_pose->trans;
  init_rot = Eigen::Quaterniond(m_relocalization_init_pose->rot);
  m_relocalization_init_pose = nullptr;
  return true;
}

CloudType::Ptr LIOLocalizationNode::getInitLidarCloud() {
  if (m_state_data.lidar_buffer.empty()) {
    LOG(WARNING) << "No lidar data received yet, waiting...";
    LOG(WARNING) << "Waiting for initial pose....";
    return nullptr;
  }

  CloudType::Ptr lidar_data = m_state_data.lidar_buffer.back().second;
  clearDataBuffer();
  return lidar_data;
}

void LIOLocalizationNode::saveLatestLidarPose() {
  M3D latest_rot_wl = m_builder->lidar_processor()->r_wl();
  V3D latest_t_wl = m_builder->lidar_processor()->t_wl();
  Eigen::Quaterniond q(latest_rot_wl);

  double delta_trans = (m_latest_pos - latest_t_wl).norm();
  double delta_deg = m_latest_rot.angularDistance(q) * 57.324;
  if (delta_trans < 0.2 && delta_deg < 10) {
    return;
  }
  m_latest_pos = latest_t_wl;
  m_latest_rot = q;

  // 要求位姿文件所在的文件夹必须存在
  std::ofstream ofs(m_saved_pose_file_path);
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  double time = static_cast<double>(now_c);

  if (ofs.is_open()) {
    ofs << std::fixed << std::setprecision(6) << time << " " << latest_t_wl.x() << " " << latest_t_wl.y() << " "
        << latest_t_wl.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    ofs.close();
  }
}

void LIOLocalizationNode::readSavedPose(const std::string& file_path) {
  if (!std::filesystem::exists(file_path)) {
    LOG(INFO) << "Not find saved pose file";
    m_try_saved_pose = false;
    return;
  }

  // 要求保存的位姿在第一行
  std::ifstream ifs(file_path);
  std::string line;
  std::getline(ifs, line);
  std::stringstream ss(line);

  // time x y z qx qy qz qw
  std::vector<std::string> tokens;
  std::string token;
  while (std::getline(ss, token, ' ')) {
    tokens.push_back(token);
  }

  CHECK(tokens.size() == 8) << "not valid pose file";
  double time = std::stod(tokens[0]);
  m_saved_pos = V3D(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
  m_saved_rot =
      Eigen::Quaterniond(std::stod(tokens[7]), std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]));

  // 把读取的位姿记录下来，用于更新
  m_latest_pos = m_saved_pos;
  m_latest_rot = m_saved_rot;
}

void LIOLocalizationNode::relocalization() {
  Eigen::Vector3d init_pos;
  Eigen::Quaterniond init_rot;

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!getInitPose(init_pos, init_rot)) {
      return;
    }
  }

  CloudType::Ptr lidar_data = nullptr;
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    lidar_data = getInitLidarCloud();
    if (lidar_data == nullptr) {
      return;
    }
  }

  // 进行icp匹配
  M4F transform_global_local = M4F::Identity();
  transform_global_local.topRightCorner(3, 1) = init_pos.cast<float>();
  transform_global_local.topLeftCorner(3, 3) = init_rot.toRotationMatrix().cast<float>();
  m_icp_localizer->setInput(lidar_data);

  // 默认第一个初值是从文件中读取的
  m_try_saved_pose = false;

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

    // 清空数据缓存，避免重定位期间数据堆积过多
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      clearDataBuffer();
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "ICP not match, waiting for new initial pose....");
}

void LIOLocalizationNode::clearDataBuffer() {
  m_state_data.lidar_pushed = false;
  m_state_data.lidar_buffer.clear();
  m_state_data.imu_buffer.clear();
  m_state_data.path.poses.clear();
  m_state_data.last_imu_time = -1.0;
  m_state_data.last_lidar_time = -1.0;
}