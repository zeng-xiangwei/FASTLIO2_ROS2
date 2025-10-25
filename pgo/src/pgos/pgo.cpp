#include "pgo.h"
#include <glog/logging.h>
PGO::PGO(const PgoConfig &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    // 设置重新线性化阈值，与窗口大小相关
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
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

    // m_icp.setMaximumIterations(50);
    // m_icp.setMaxCorrespondenceDistance(10);
    // m_icp.setTransformationEpsilon(1e-6);
    // m_icp.setEuclideanFitnessEpsilon(1e-6);
    // m_icp.setRANSACIterations(0);

    // global_icp.setMaximumIterations(50);
    // global_icp.setMaxCorrespondenceDistance(10);
    // global_icp.setTransformationEpsilon(1e-6);
    // global_icp.setEuclideanFitnessEpsilon(1e-6);
    // global_icp.setRANSACIterations(0);

    global_idx = SIZE_MAX;
    global_map_load = false;
    key_size_all = 0;
    // 加载初始位姿，目前是通过rviz指定的，所这这里暂时不需要
    // loadPose();
    m_icp_localizer = std::make_shared<ICPLocalizer>(m_config.icp_config);
    m_key_poses.reserve(m_config.max_key_poses);
}


void PGO::loadPose()
{
    if (m_config.pose_load_mode == 0)
    {
        initial_pose_r = Eigen::AngleAxisd(m_config.initial_pose_r(0), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(m_config.initial_pose_r(1), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(m_config.initial_pose_r(2), Eigen::Vector3d::UnitZ());
        // initial_pose_r = m_config.initial_pose_r;
        initial_pose_t = m_config.initial_pose_t;
        
    } else if(m_config.pose_load_mode == 1)
    {
        std::ifstream pose_file(m_config.initial_pose_file);
        if (!pose_file.is_open())
        {
            std::cout << "Failed to open pose file: " << m_config.initial_pose_file << std::endl;
            return;
        }
        V3D initial_pose_r_eular = V3D::Zero();
        pose_file >> initial_pose_r_eular(0) >> initial_pose_r_eular(1) >> initial_pose_r_eular(2)
                >> initial_pose_t(0) >> initial_pose_t(1) >> initial_pose_t(2);
        initial_pose_r = Eigen::AngleAxisd(initial_pose_r_eular(0), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(initial_pose_r_eular(1), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(initial_pose_r_eular(2), Eigen::Vector3d::UnitZ());

        std::cout << "initial pose: " << initial_pose_r.transpose() << " " 
                    << initial_pose_t.transpose() << std::endl;
    }
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

bool PGO::initialPose(const CloudWithPose &cloud_with_pose, 
    const Eigen::Vector3d &init_pose_t, const Eigen::Quaterniond &init_pose_r)
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

    m_icp_localizer->setInput(cloud_with_pose.cloud);
    M4F transform_global_local = M4F::Identity();
    transform_global_local.topRightCorner(3, 1) = init_pose_t.cast<float>();
    transform_global_local.topLeftCorner(3, 3) = init_pose_r.toRotationMatrix().cast<float>();

    if(m_icp_localizer->align(transform_global_local)) {
        // update offset by icp
        m_r_offset = transform_global_local.block<3, 3>(0, 0).cast<double>();
        m_t_offset = transform_global_local.block<3, 1>(0, 3).cast<double>();
    }

    // todo: how to set thresh 
    // 比较m_r_offset与initial_pose_r
    if ((m_r_offset - initial_pose_r).norm() > m_config.angle_thresh)
    {
        std::cout << "initial localization failed."<< std::endl;
        std::cout << "m_r_offset: " << m_r_offset.transpose() << "\n" 
                    << initial_pose_r.transpose() << std::endl;
        return false;
    }
    // 比较m_t_offset与initial_pose_t
    if ((m_t_offset - initial_pose_t).norm() > m_config.trans_thresh)
    {
        std::cout << "initial localization failed."<< std::endl;
        std::cout << "m_t_offset: " << m_t_offset.transpose() << "\n" 
                    << initial_pose_t.transpose() << std::endl;
        return false;
    }

    return true;
}

bool PGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    // if(m_config.model == "mapping")
    // {
    //     if (!is_key_pose)
    //         return false;
    // }
    if (!is_key_pose)
        return false;
    // size_t idx = m_key_poses.size();
    size_t idx = key_size_all;
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = 
            gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, 
            gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, 
            gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);
    key_size_all++;
    // 如果m_key_poses的size大于某个数量，删除一半的最早的key pose,这个只在定位的时候会生效
    // if (m_key_poses.size() > static_cast<size_t>(m_config.max_key_poses))
    if (m_key_poses.size() > m_config.max_key_poses && m_config.model == "localization")
        m_key_poses.erase(m_key_poses.begin(), m_key_poses.begin() + m_config.max_key_poses/2);
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
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, 
            Eigen::Quaterniond(m_key_poses[i].r_global));
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

    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, 
        m_config.submap_resolution);
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
    one_pair.target_id = loop_idx;
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
    if (m_config.model == "localization" && m_config.match_enable)
    {
        GlobalMatch();
    }
    else if (m_config.model == "mapping")
    {
        searchForLoopPairs();
    }

    smoothAndUpdate();
    
}
void PGO::GlobalMatch()
{
    // m_icp_localizer->setInput(cloud_with_pose.cloud);
    m_icp_localizer->setInput(m_key_poses.back().body_cloud);

    // double转float会有损失？
    M4F transform_global_local = M4F::Identity();
    transform_global_local.topRightCorner(3, 1) = m_key_poses.back().t_global.cast<float>();
    transform_global_local.topLeftCorner(3, 3) = m_key_poses.back().r_global.cast<float>();  
    // 打印匹配需要的耗时
    auto start = std::chrono::steady_clock::now();
    if(m_icp_localizer->align(transform_global_local)) {
        // update offset by icp
        m_r_offset = transform_global_local.block<3, 3>(0, 0).cast<double>();
        m_t_offset = transform_global_local.block<3, 1>(0, 3).cast<double>();
    }
    // 打印匹配耗时
    auto end = std::chrono::steady_clock::now();
    std::cout << "GlobalMatch time: " 
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
        << " ms" << std::endl;

    std::cout << "GlobalMatch score: " << m_icp_localizer->getRefineScore() << std::endl;
    std::cout << "GlobalMatch r_offset: " << m_r_offset << std::endl;
    std::cout << "GlobalMatch t_offset: " << m_t_offset << std::endl;

    // 添加全局地图的初始位姿，都是0
    if(!have_add_global_pose)
    {
        m_initial_values.insert(global_idx, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3()));
        have_add_global_pose = true;
    }

    // size_t cur_idx = m_key_poses.size() - 1;
    size_t cur_idx = key_size_all - 1;
    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = global_idx;
    one_pair.score = m_icp_localizer->getRefineScore();
    one_pair.r_offset = m_r_offset;
    one_pair.t_offset= m_t_offset;
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
    // 打印isam2的优化耗时
    auto start = std::chrono::steady_clock::now();
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
    // 打印isam2的优化耗时，按照微妙打印
    auto end = std::chrono::steady_clock::now();
    std::cout << "isam2 time: " 
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() 
        << " us" << std::endl;

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
    // std::cout << "r_offset: " << m_r_offset << std::endl;
    // std::cout << "t_offset: " << m_t_offset << std::endl;
}

void PGO::loadMap()
{

    global_map_load = m_icp_localizer->loadMap(m_config.global_pcd_file);
    // global_map_load = true;
    // return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PGO::getMapCloud()
{
    // map_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    return m_icp_localizer->refineMap();
}