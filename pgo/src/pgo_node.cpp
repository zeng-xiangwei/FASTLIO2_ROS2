#include "pgo_node.h"
#include <glog/logging.h>

using namespace std::chrono_literals;

PGONode::PGONode(const std::string& node_name) : Node(node_name) {
  RCLCPP_INFO(this->get_logger(), "PGO node started");

  loadParameters();

  // m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
  m_pgo = std::make_shared<PGO>(m_pgo_config);
  rclcpp::QoS qos = rclcpp::QoS(1);
  m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
  m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
  m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10);

  m_global_map_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", rclcpp::QoS(1).transient_local());

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  m_sync = std::make_shared<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
      m_cloud_sub, m_odom_sub);
  m_sync->setAgePenalty(0.1);
  m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
  m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
  m_save_map_srv = this->create_service<interface::srv::SaveMaps>(
      "/pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));

  m_relocalization_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1, std::bind(&PGONode::initPoseCB, this, std::placeholders::_1));
    
  m_pgo->initial();
}

void PGONode::loadParameters() {
  this->declare_parameter("config_path", "");
  std::string config_path;
  this->get_parameter<std::string>("config_path", config_path);
  YAML::Node config = YAML::LoadFile(config_path);
  if (!config) {
    RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
  m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
  m_node_config.odom_topic = config["odom_topic"].as<std::string>();
  m_node_config.map_frame = config["map_frame"].as<std::string>();
  m_node_config.local_frame = config["local_frame"].as<std::string>();

  // 从ROS参数读取initial_pose_file和global_pcd_file（如果未设置则为空字符串）
  this->declare_parameter("initial_pose_file", "");
  this->declare_parameter("global_pcd_file", "");
  this->declare_parameter("pose_load_mode", -1);
  std::string initial_pose_file, global_pcd_file;
  int pose_load_mode;
  this->get_parameter<std::string>("initial_pose_file", initial_pose_file);
  this->get_parameter<std::string>("global_pcd_file", global_pcd_file);
  this->get_parameter<int>("pose_load_mode", pose_load_mode);
  m_pgo_config.initial_pose_file = initial_pose_file;
  m_pgo_config.global_pcd_file = global_pcd_file;

  YAML::Node pgo_config = config["pgo_config"];
  // 如果 ros 参数中设定了初始位姿读取方式，则用，否则用yaml中的配置
  m_pgo_config.pose_load_mode = pose_load_mode >= 0 ? pose_load_mode : pgo_config["pose_load_mode"].as<int>();

  m_pgo_config.key_pose_delta_deg = pgo_config["key_pose_delta_deg"].as<double>();
  m_pgo_config.key_pose_delta_trans = pgo_config["key_pose_delta_trans"].as<double>();
  m_pgo_config.loop_search_radius = pgo_config["loop_search_radius"].as<double>();
  m_pgo_config.loop_time_tresh = pgo_config["loop_time_tresh"].as<double>();
  m_pgo_config.loop_score_tresh = pgo_config["loop_score_tresh"].as<double>();
  m_pgo_config.loop_submap_half_range = pgo_config["loop_submap_half_range"].as<int>();
  m_pgo_config.submap_resolution = pgo_config["submap_resolution"].as<double>();
  m_pgo_config.min_loop_detect_duration = pgo_config["min_loop_detect_duration"].as<double>();
  m_pgo_config.global_score_tresh = pgo_config["global_score_tresh"].as<double>();
  m_pgo_config.model = pgo_config["model"].as<std::string>();
  m_pgo_config.match_enable = pgo_config["match_enable"].as<bool>();
  m_pgo_config.angle_thresh = pgo_config["angle_thresh"].as<double>();
  m_pgo_config.trans_thresh = pgo_config["trans_thresh"].as<double>();
  m_pgo_config.max_key_poses = pgo_config["max_key_poses"].as<int>();
  
  YAML::Node icp_config = pgo_config["icp_config"];
  m_pgo_config.icp_config.rough_scan_resolution = icp_config["rough_scan_resolution"].as<double>();
  m_pgo_config.icp_config.rough_map_resolution = icp_config["rough_map_resolution"].as<double>();
  m_pgo_config.icp_config.rough_max_iteration = icp_config["rough_max_iteration"].as<int>();
  m_pgo_config.icp_config.rough_score_thresh = icp_config["rough_score_thresh"].as<double>();
  m_pgo_config.icp_config.rough_score_dis_thresh = icp_config["rough_score_dis_thresh"].as<double>();

  m_pgo_config.icp_config.refine_scan_resolution = icp_config["refine_scan_resolution"].as<double>();
  m_pgo_config.icp_config.refine_map_resolution = icp_config["refine_map_resolution"].as<double>();
  m_pgo_config.icp_config.refine_max_iteration = icp_config["refine_max_iteration"].as<int>();
  m_pgo_config.icp_config.refine_score_thresh = icp_config["refine_score_thresh"].as<double>();
  m_pgo_config.icp_config.refine_score_dis_thresh = icp_config["refine_score_dis_thresh"].as<double>();

  YAML::Node grid_map_config = config["grid_map_config"];
  m_grid_map_config.grid_2d_z_min = grid_map_config["grid_2d_z_min"].as<float>();
  m_grid_map_config.grid_2d_z_max = grid_map_config["grid_2d_z_max"].as<float>();
  m_grid_map_config.grid_2d_resolution = grid_map_config["grid_2d_resolution"].as<float>();
  m_grid_map_config.occupancy_weight = grid_map_config["occupancy_weight"].as<int>();
}


void PGONode::syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
            const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
  std::lock_guard<std::mutex> lock(m_state.message_mutex);
  CloudWithPose cp;
  cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
  if (cp.pose.second < m_state.last_message_time) {
    RCLCPP_WARN(this->get_logger(), "Received out of order message");
    return;
  }
  m_state.last_message_time = cp.pose.second;

  cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                                  odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
                  .toRotationMatrix();
  cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
  cp.cloud = CloudType::Ptr(new CloudType);
  pcl::fromROSMsg(*cloud_msg, *cp.cloud);
  m_state.cloud_buffer.push(cp);
}

void PGONode::sendBroadCastTF(builtin_interfaces::msg::Time& time) {
  // std::cout << "-------------send tf ----------------" << std::endl;
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = m_node_config.map_frame;
  transformStamped.child_frame_id = m_node_config.local_frame;
  transformStamped.header.stamp = time;
  Eigen::Quaterniond q(m_pgo->offsetR());
  V3D t = m_pgo->offsetT();
  // std::cout << "q: " << q.toRotationMatrix() << "\n" <<" t: " << t.transpose() << std::endl;
  transformStamped.transform.translation.x = t.x();
  transformStamped.transform.translation.y = t.y();
  transformStamped.transform.translation.z = t.z();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  m_tf_broadcaster->sendTransform(transformStamped);
}

void PGONode::publishLoopMarkers(builtin_interfaces::msg::Time& time) {
  if (m_loop_marker_pub->get_subscription_count() == 0) return;
  // here return if localization model
  if (m_pgo->historyPairs().size() == 0) return;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker nodes_marker;
  visualization_msgs::msg::Marker edges_marker;
  nodes_marker.header.frame_id = m_node_config.map_frame;
  nodes_marker.header.stamp = time;
  nodes_marker.ns = "pgo_nodes";
  nodes_marker.id = 0;
  nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes_marker.action = visualization_msgs::msg::Marker::ADD;
  nodes_marker.pose.orientation.w = 1.0;
  nodes_marker.scale.x = 0.3;
  nodes_marker.scale.y = 0.3;
  nodes_marker.scale.z = 0.3;
  nodes_marker.color.r = 1.0;
  nodes_marker.color.g = 0.8;
  nodes_marker.color.b = 0.0;
  nodes_marker.color.a = 1.0;

  edges_marker.header.frame_id = m_node_config.map_frame;
  edges_marker.header.stamp = time;
  edges_marker.ns = "pgo_edges";
  edges_marker.id = 1;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.pose.orientation.w = 1.0;
  edges_marker.scale.x = 0.1;
  edges_marker.color.r = 0.0;
  edges_marker.color.g = 0.8;
  edges_marker.color.b = 0.0;
  edges_marker.color.a = 1.0;

  std::vector<KeyPoseWithCloud>& poses = m_pgo->keyPoses();
  std::vector<std::pair<size_t, size_t>>& pairs = m_pgo->historyPairs();
  for (size_t i = 0; i < pairs.size(); i++) {
    size_t i1 = pairs[i].first;
    size_t i2 = pairs[i].second;
    geometry_msgs::msg::Point p1, p2;
    p1.x = poses[i1].t_global.x();
    p1.y = poses[i1].t_global.y();
    p1.z = poses[i1].t_global.z();

    p2.x = poses[i2].t_global.x();
    p2.y = poses[i2].t_global.y();
    p2.z = poses[i2].t_global.z();

    nodes_marker.points.push_back(p1);
    nodes_marker.points.push_back(p2);
    edges_marker.points.push_back(p1);
    edges_marker.points.push_back(p2);
  }

  marker_array.markers.push_back(nodes_marker);
  marker_array.markers.push_back(edges_marker);
  m_loop_marker_pub->publish(marker_array);
}

void PGONode::timerCB() {
  if (m_state.cloud_buffer.size() == 0) return;
  CloudWithPose cp = m_state.cloud_buffer.front();
  // 清理队列
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    while (!m_state.cloud_buffer.empty()) {
      m_state.cloud_buffer.pop();
    }
  }
  // M3D initial_pose_r; V3D initial_pose_t;
  Eigen::Vector3d init_pos;
  Eigen::Quaterniond init_rot;
  // get time of current cloud points
  builtin_interfaces::msg::Time cur_time;
  cur_time.sec = cp.pose.sec;
  cur_time.nanosec = cp.pose.nsec;

  if(!initial_state &&  m_pgo_config.model == "localization")
  {
    //加载地图并发布到rviz
    if(m_pgo->getGlobalMapLoadStatus()) {
      std::cout << "Load global map."  << std::endl;
      publishGlobalMap(m_pgo->getMapCloud());
    }

    if(!getInitPose(init_pos, init_rot)) {
      return;
    }
    if(!m_pgo->initialPose(cp, init_pos, init_rot)) {
      return;
    }

    std::cout << "initial pose success." << std::endl;
    std::cout << "initial pose: " << init_pos.transpose() << " " 
        << init_rot.coeffs().transpose() << std::endl;
    initial_state = true;
    sendBroadCastTF(cur_time);
  }

  //test
  // if (true) {
  //   sendBroadCastTF(cur_time);
  //   return;
  // }

  if (!m_pgo->addKeyPose(cp)) {
    sendBroadCastTF(cur_time);
    return;
  }
  // 后端只对关键帧进行匹配
  m_pgo->Match();

  sendBroadCastTF(cur_time);

  publishLoopMarkers(cur_time);
}

void PGONode::saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request,
                std::shared_ptr<interface::srv::SaveMaps::Response> response) {
  if (!std::filesystem::exists(request->file_path)) {
    response->success = false;
    response->message = request->file_path + " IS NOT EXISTS!";
    return;
  }

  if (m_pgo->keyPoses().size() == 0) {
    response->success = false;
    response->message = "NO POSES!";
    return;
  }

  std::filesystem::path p_dir(request->file_path);
  std::filesystem::path patches_dir = p_dir / "patches";
  std::filesystem::path poses_txt_path = p_dir / "poses.txt";
  std::filesystem::path map_path = p_dir / "map.pcd";

  if (request->save_patches) {
    if (std::filesystem::exists(patches_dir)) {
      std::filesystem::remove_all(patches_dir);
    }

    std::filesystem::create_directories(patches_dir);

    if (std::filesystem::exists(poses_txt_path)) {
      std::filesystem::remove(poses_txt_path);
    }
    RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
  }
  RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

  std::ofstream txt_file(poses_txt_path);

  CloudType::Ptr ret(new CloudType);

  utils::OccupancyMap::Config config;
  config.resolution = m_grid_map_config.grid_2d_resolution;
  config.min_z = m_grid_map_config.grid_2d_z_min;
  config.max_z = m_grid_map_config.grid_2d_z_max;
  config.occupancy_weight = m_grid_map_config.occupancy_weight;
  utils::OccupancyMap occupancy_map(config);
  for (size_t i = 0; i < m_pgo->keyPoses().size(); i++) {
    CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
    if (request->save_patches) {
      std::string patch_name = std::to_string(i) + ".pcd";
      std::filesystem::path patch_path = patches_dir / patch_name;
      pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
      Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
      V3D t = m_pgo->keyPoses()[i].t_global;
      txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " "
                << q.y() << " " << q.z() << std::endl;
    }
    CloudType::Ptr world_cloud(new CloudType);
    pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global,
                              Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
    *ret += *world_cloud;
    occupancy_map.AddLidarFrame(body_cloud, m_pgo->keyPoses()[i].t_global,
        Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
  }
  txt_file.close();
  pcl::io::savePCDFileBinary(map_path.string(), *ret);

  // 保存2d栅格地图
  occupancy_map.Save(map_path.parent_path().string(), "map_2d");

  response->success = true;
  response->message = "SAVE SUCCESS!";
}

void PGONode::initPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // std::lock_guard<std::mutex> lock(m_mutex);
  m_relocalization_init_pose = std::make_shared<Pose>();
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z);
  m_relocalization_init_pose->rot = M3D(q);
  m_relocalization_init_pose->trans =
      V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

bool PGONode::getInitPose(Eigen::Vector3d& init_pos, Eigen::Quaterniond& init_rot) {
  // pose_load_mode == 1: 先尝试从文件读取，失败则回退到话题
  if (m_pgo_config.pose_load_mode == 1 && !m_pose_file_loaded) {
    // 尝试从文件读取初始位姿
    if (!m_pgo_config.initial_pose_file.empty()) {
      std::ifstream pose_file(m_pgo_config.initial_pose_file);
      if (pose_file.is_open()) {
        double timestamp, x, y, z, qx, qy, qz, qw;
        if (pose_file >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw) {
          init_pos = Eigen::Vector3d(x, y, z);
          init_rot = Eigen::Quaterniond(qw, qx, qy, qz);
          m_pose_file_loaded = true;
          LOG(INFO) << "Loaded initial pose from file: " << m_pgo_config.initial_pose_file;
          LOG(INFO) << "Initial pose: " << init_pos.transpose() << " " << init_rot.coeffs().transpose();
          pose_file.close();
          return true;
        }
        pose_file.close();
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to open initial pose file: %s", m_pgo_config.initial_pose_file.c_str());
      }
    }
    // 文件读取失败，标记已尝试过，避免重复尝试
    m_pose_file_loaded = true;
  }

  // pose_load_mode == 0 或 文件读取失败，从话题获取
  if (m_relocalization_init_pose == nullptr) {
    return false;
  }
  LOG(INFO) << "Get initial pose from topic!";
  init_pos = m_relocalization_init_pose->trans;
  init_rot = Eigen::Quaterniond(m_relocalization_init_pose->rot);
  m_relocalization_init_pose = nullptr;
  return true;
}


void PGONode::publishGlobalMap(CloudType::Ptr cloud) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = m_node_config.map_frame;
  cloud_msg.header.stamp = this->get_clock()->now();
  m_global_map_pub->publish(cloud_msg);
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PGONode>("pgo_node"));
  rclcpp::shutdown();
  return 0;
}