#include "pgo.h"

PGO::PGO(const Config &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    // 设置重新线性化阈值，与窗口大小相关
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    // 设置优化参数
    isam2_params.optimizationParams = gtsam::ISAM2DoglegParams::Create(
        gtsam::ISAM2DoglegParams::TrustRegionAdaptationMode::SEARCH);
    // 设置评估非线性误差
    isam2_params.evaluateNonlinearError = false;
    // 启用详细结果输出
    isam2_params.enableDetailedResults = false;
    // ISAM2 增量式优化
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(10);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);

    global_icp.setMaximumIterations(50);
    global_icp.setMaxCorrespondenceDistance(10);
    global_icp.setTransformationEpsilon(1e-6);
    global_icp.setEuclideanFitnessEpsilon(1e-6);
    global_icp.setRANSACIterations(0);

    global_idx = SIZE_MAX;
    global_map_load = false;
}

bool PGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}

bool PGO::initialPose(const CloudWithPose &cloud_with_pose)
{
    if(m_config.model == "mapping")
    {
        return true;
    }

    if (!global_map_load)
    {
        // std::cout << "Load global map from " << m_config.global_pcd_file << std::endl;
        loadMap();
    }

    if (!map_cloud)
    {
        std::cout << "current map cloud is empty" << std::endl;
        return false;
    }

    // only used the current frame
    // CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);

    CloudType::Ptr body_cloud = cloud_with_pose.cloud;
    CloudType::Ptr global_cloud(new CloudType);
    pcl::transformPointCloud(*body_cloud *global_cloud, initial_pose.t_global, 
        Eigen::Quaterniond(initial_pose.r_global));

    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    global_icp.setInputSource(global_cloud);
    global_icp.setInputTarget(map_cloud);
    global_icp.align(*align_cloud);
    if (!global_icp.hasConverged() || global_icp.getFitnessScore() > m_config.global_score_tresh)
        return;
    M4F global_transform = global_icp.getFinalTransformation();

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    // update initial pose by icp
    m_r_offset = global_transform.block<3, 3>(0, 0).cast<double>() * initial_pose.r_global;
    m_t_offset = global_transform.block<3, 3>(0, 0).cast<double>() * initial_pose.t_global + 
        global_transform.block<3, 1>(0, 3).cast<double>();

    return true;
}

bool PGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);
    return true;
}

CloudType::Ptr PGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

void PGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    pcl::PointXYZ last_pose_pt;
    last_pose_pt.x = last_item.t_global(0);
    last_pose_pt.y = last_item.t_global(1);
    last_pose_pt.z = last_item.t_global(2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < m_key_poses.size() - 1; i++)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        key_poses_cloud->push_back(pt);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(key_poses_cloud);
    std::vector<int> ids;
    std::vector<float> sqdists;
    int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
    if (neighbors == 0)
        return;

    int loop_idx = -1;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int idx = ids[i];
        if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh)
        {
            loop_idx = idx;
            break;
        }
    }

    if (loop_idx == -1)
        return;

    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);
    CloudType::Ptr align_cloud(new CloudType);

    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);
    m_icp.align(*align_cloud);

    if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh)
        return;

    M4F loop_transform = m_icp.getFinalTransformation();

    LoopPair one_pair;
    
    one_pair.source_id = cur_idx;
    one_pair.target_id = global_idx;
    one_pair.score = m_icp.getFitnessScore();
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + 
        loop_transform.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * 
        (t_refined - m_key_poses[loop_idx].t_global);
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void PGO::Match() {
    if (m_config.model == "mapping")
    {
        GlobalMatch();
    }
    else if (m_config.model == "localization")
    {
        searchForLoopPairs();
    }

    smoothAndUpdate();

    // if(m_config.model == "localization")
    // {

    // }
        
}
void PGO::GlobalMatch()
{
    if (!map_cloud)
        return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // only used the current frame
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);

    global_icp.setInputSource(source_cloud);
    global_icp.setInputTarget(map_cloud);
    global_icp.align(*align_cloud);
    if (!global_icp.hasConverged() || global_icp.getFitnessScore() > m_config.global_score_tresh)
        return;
    M4F global_transform = global_icp.getFinalTransformation();

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    M3D r_refined = global_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = global_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + 
        global_transform.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * 
        (t_refined - m_key_poses[loop_idx].t_global);
    m_cache_pairs.push_back(one_pair);
}

void PGO::smoothAndUpdate()
{
    bool has_loop_or_global_match = !m_cache_pairs.empty();
    // 添加回环因子or全局匹配因子
    if (has_loop_or_global_match)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                gtsam::Pose3(gtsam::Rot3(pair.r_offset),gtsam::Point3(pair.t_offset)),
                gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop_or_global_match)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}

void PGO::loadMap()
{
    if (m_config.global_pcd_file.empty())
    {
        std::cout << "Global map file is empty" <<std::endl;
        return false;
    }
    map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(m_config.global_pcd_file, *map_cloud) == -1)
    {
        std::cerr << "Failed to load map cloud from " << m_config.global_pcd_file << std::endl;
        return false;
    }
    global_map_load = true;
    // return true;
}
