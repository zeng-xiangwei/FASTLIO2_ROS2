#include "lio_node.h"

#include <glog/logging.h>

LIONode::LIONode(const std::string& node_name) : Node(node_name) { m_thread = std::thread(&LIONode::loopThread, this); }

LIONode::~LIONode() {
  m_finished = true;
  m_condition.notify_all();
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void LIONode::initRos() {
  RCLCPP_INFO(this->get_logger(), "%s Started", this->get_name());
  loadParameters();

  m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 100,
                                                               std::bind(&LIONode::imuCB, this, std::placeholders::_1));
  if (m_node_config.lidar_type == kLivoxLidarType) {
    m_livox_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        m_node_config.lidar_topic, 1, std::bind(&LIONode::livoxLidarCB, this, std::placeholders::_1));
  } else if (m_node_config.lidar_type == kRobosenseLidarType) {
    m_robosense_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        m_node_config.lidar_topic, 1, std::bind(&LIONode::robosenseLidarCB, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Lidar type error, please check lidar_type");
  }

  m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 1);
  m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 1);
  m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 1);
  m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10);
  m_imu_frec_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_imu_frec_odom", 10);
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  m_state_data.path.poses.clear();
  m_state_data.path.header.frame_id = m_node_config.world_frame;

  m_kf = std::make_shared<IESKF>();
  m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
  m_imu_pose_predictor = std::make_shared<ImuPosePredictor>();
  m_imu_freq_timer = this->create_wall_timer(10ms, std::bind(&LIONode::imuFreqCB, this));
}

void LIONode::loadParameters() {
  LOG(INFO) << "in LIONode loadPrama";
  this->declare_parameter("config_path", "");
  std::string config_path;
  this->get_parameter<std::string>("config_path", config_path);

  YAML::Node config = YAML::LoadFile(config_path);
  if (!config) {
    RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

  m_node_config.imu_topic = config["imu_topic"].as<std::string>();
  m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
  m_node_config.body_frame = config["body_frame"].as<std::string>();
  m_node_config.world_frame = config["world_frame"].as<std::string>();
  m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
  m_node_config.ros_spin_thread = config["ros_spin_thread"].as<int>();
  m_node_config.lidar_type = config["lidar_type"].as<std::string>();

  if ((m_node_config.lidar_type != kLivoxLidarType) && (m_node_config.lidar_type != kRobosenseLidarType)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid lidar type: %s", m_node_config.lidar_type.c_str());
  }

  if (m_node_config.lidar_type == kRobosenseLidarType && config["imu_data_preprocess_rot"]) {
    std::vector<double> tmp_rot_vec = config["imu_data_preprocess_rot"].as<std::vector<double>>();
    m_node_config.imu_data_preprocess_rot << tmp_rot_vec[0], tmp_rot_vec[1], tmp_rot_vec[2], tmp_rot_vec[3],
        tmp_rot_vec[4], tmp_rot_vec[5], tmp_rot_vec[6], tmp_rot_vec[7], tmp_rot_vec[8];
  }
  if (config["n_scans"]) {
    m_node_config.n_scans = config["n_scans"].as<int>();
  }

  m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
  m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
  m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
  m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
  m_builder_config.map_resolution = config["map_resolution"].as<double>();
  m_builder_config.cube_len = config["cube_len"].as<double>();
  m_builder_config.det_range = config["det_range"].as<double>();
  m_builder_config.move_thresh = config["move_thresh"].as<double>();
  m_builder_config.na = config["na"].as<double>();
  m_builder_config.ng = config["ng"].as<double>();
  m_builder_config.nba = config["nba"].as<double>();
  m_builder_config.nbg = config["nbg"].as<double>();

  m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
  m_builder_config.near_search_num = config["near_search_num"].as<int>();
  m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
  m_builder_config.gravity_align = config["gravity_align"].as<bool>();
  m_builder_config.esti_il = config["esti_il"].as<bool>();
  std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
  std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
  m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
  m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6],
      r_il_vec[7], r_il_vec[8];
  m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
}

void LIONode::imuCB(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(m_mutex);
  double timestamp = Utils::getSec(msg->header);
  if (timestamp < m_state_data.last_imu_time) {
    RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
    std::deque<IMUData>().swap(m_state_data.imu_buffer);
  }

  V3D acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0;
  V3D gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  // 考虑到 imu 与 lidar 坐标系不平行时（robosense airy），将 imu 数据转到与 lidar 平行的坐标系下，否则地图是倒着的
  acc = m_node_config.imu_data_preprocess_rot * acc;
  gyro = m_node_config.imu_data_preprocess_rot * gyro;

  m_state_data.imu_buffer.emplace_back(acc, gyro, timestamp);
  m_state_data.last_imu_time = timestamp;

  m_imu_pose_predictor->addImuData(m_state_data.imu_buffer.back());
  m_state_data.has_new_data = true;
  m_condition.notify_all();
}
void LIONode::livoxLidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
                                          m_builder_config.lidar_max_range);
  std::lock_guard<std::mutex> lock(m_mutex);
  double timestamp = Utils::getSec(msg->header);
  if (timestamp < m_state_data.last_lidar_time) {
    RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
  }
  m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
  m_state_data.last_lidar_time = timestamp;
  m_state_data.has_new_data = true;
  m_condition.notify_all();
}

void LIONode::robosenseLidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto t1 = std::chrono::high_resolution_clock::now();
  CloudType::Ptr cloud = Utils::robosense2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
                                              m_builder_config.lidar_max_range, m_node_config.n_scans);
  auto t2 = std::chrono::high_resolution_clock::now();

  double cost = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
  RCLCPP_WARN(this->get_logger(), "get robosense msg cost %f", cost);
  std::lock_guard<std::mutex> lock(m_mutex);
  double timestamp = Utils::getSec(msg->header);
  if (timestamp < m_state_data.last_lidar_time) {
    RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
  }
  m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
  m_state_data.last_lidar_time = timestamp;
  m_state_data.has_new_data = true;
  m_condition.notify_all();
}

bool LIONode::syncPackage() {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty()) return false;
  if (!m_state_data.lidar_pushed) {
    m_package.cloud = m_state_data.lidar_buffer.front().second;
    std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(),
              [](PointType& p1, PointType& p2) { return p1.curvature < p2.curvature; });
    m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
    m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
    m_state_data.lidar_pushed = true;
  }
  if (m_state_data.last_imu_time < m_package.cloud_end_time) return false;

  Vec<IMUData>().swap(m_package.imus);
  while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time) {
    m_package.imus.emplace_back(m_state_data.imu_buffer.front());
    m_state_data.imu_buffer.pop_front();
  }
  m_state_data.lidar_buffer.pop_front();
  m_state_data.lidar_pushed = false;
  return true;
}

void LIONode::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud,
                           std::string frame_id, const double& time) {
  if (pub->get_subscription_count() <= 0) return;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.header.stamp = Utils::getTime(time);
  pub->publish(cloud_msg);
}

void LIONode::publishLiOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
                                std::string child_frame, const double& time) {
  V3D trans = m_kf->x().t_wi;
  V3D vel = m_kf->x().v;
  M3D rot = m_kf->x().r_wi;
  publishOdometry(odom_pub, frame_id, child_frame, time, trans, rot, vel);
}

void LIONode::publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
                              std::string child_frame, const double& time, const V3D& trans, const M3D& rot,
                              const V3D& vel) {
  if (odom_pub->get_subscription_count() <= 0) return;
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = frame_id;
  odom.header.stamp = Utils::getTime(time);
  odom.child_frame_id = child_frame;
  odom.pose.pose.position.x = trans.x();
  odom.pose.pose.position.y = trans.y();
  odom.pose.pose.position.z = trans.z();
  Eigen::Quaterniond q(rot);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = vel.x();
  odom.twist.twist.linear.y = vel.y();
  odom.twist.twist.linear.z = vel.z();
  odom_pub->publish(odom);
}

void LIONode::publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id,
                          const double& time) {
  if (path_pub->get_subscription_count() <= 0) return;

  V3D trans = m_kf->x().t_wi;
  M3D rot = m_kf->x().r_wi;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = Utils::getTime(time);
  pose.pose.position.x = trans.x();
  pose.pose.position.y = trans.y();
  pose.pose.position.z = trans.z();
  Eigen::Quaterniond q(rot);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  m_state_data.path.poses.push_back(pose);
  path_pub->publish(m_state_data.path);
}

void LIONode::broadLiCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
                            std::string child_frame, const double& time) {
  broadCastTF(broad_caster, frame_id, child_frame, time, m_kf->x().t_wi, m_kf->x().r_wi);
}

void LIONode::broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
                          std::string child_frame, const double& time, const V3D& t, const M3D& rot) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame;
  transformStamped.header.stamp = Utils::getTime(time);
  transformStamped.transform.translation.x = t.x();
  transformStamped.transform.translation.y = t.y();
  transformStamped.transform.translation.z = t.z();
  Eigen::Quaterniond q(rot);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  broad_caster->sendTransform(transformStamped);
}

void LIONode::loopThread() {
  while (true) {
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_condition.wait(lock, [this]() -> bool { return m_state_data.has_new_data || m_finished; });
    }

    if (m_finished) {
      break;
    }

    m_state_data.has_new_data = false;

    timerCB();
  }
}
void LIONode::timerCB() {
  if (!ready()) {
    return;
  }
  if (!syncPackage()) {
    return;
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  m_builder->process(m_package);
  m_imu_pose_predictor->setLioState({m_kf->x(), m_package.cloud_end_time});
  auto t2 = std::chrono::high_resolution_clock::now();

  if (m_node_config.print_time_cost) {
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
  }

  if (m_builder->status() != BuilderStatus::MAPPING) return;

  broadLiCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

  publishLiOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

  CloudType::Ptr body_cloud =
      m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

  publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

  CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(
      m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

  publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

  publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
}

void LIONode::imuFreqCB() {
  StateWithTime state;
  if (!m_imu_pose_predictor->getPredictedState(state)) {
    return;
  }

  if (m_last_imu_frec_state == nullptr) {
    m_last_imu_frec_state = std::make_shared<StateWithTime>(state);
  }

  if (m_last_imu_frec_state->timestamp >= state.timestamp) {
    return;
  }
  *m_last_imu_frec_state = state;

  V3D trans = state.state.t_wi;
  V3D vel = state.state.v;
  M3D rot = state.state.r_wi;
  publishOdometry(m_imu_frec_odom_pub, m_node_config.world_frame, m_node_config.body_frame, state.timestamp, trans, rot,
                  vel);
}

bool LIONode::ready() { return true; }