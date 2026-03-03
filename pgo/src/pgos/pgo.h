#pragma once
#include "commons.h"
#include <stdint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "simple_pgo.h"
#include "icp_localizer.h"

struct PgoConfig
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    double loop_search_radius = 1.0;
    double loop_time_tresh = 60.0;
    double loop_score_tresh = 0.15;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;

    double global_score_tresh = 0.15;
    std::string global_pcd_file = "";
    std::string model = "map";
    bool match_enable = false;
    int pose_load_mode = 0;
    Eigen::Vector3d initial_pose_r = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_pose_t = Eigen::Vector3d::Zero();
    std::string initial_pose_file = "";
    double angle_thresh = 0.1;
    double trans_thresh = 0.1;
    size_t max_key_poses = 1000;

    ICPConfig icp_config;
};

class PGO
{
public:
    PGO(const PgoConfig &config);

    void initial();
    bool initialPose(const CloudWithPose &cloud_with_pose, const Eigen::Vector3d &init_pos, 
        const Eigen::Quaterniond &init_rot);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

    void Match();

    void loadMap();
    bool GlobalMatch();
    
    bool getGlobalMapLoadStatus() { return global_map_load; }

    pcl::PointCloud<pcl::PointXYZI>::Ptr getMapCloud();

private:
    PgoConfig m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;

    // pcl::IterativeClosestPoint<PointType, PointType> global_icp;
    size_t global_idx;
    size_t key_size_all;
    
    bool global_map_load;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
    M3D initial_pose_r;
    V3D initial_pose_t;
    double angle_thresh;
    double trans_thresh;
    bool have_add_global_pose = false;

    std::shared_ptr<ICPLocalizer> m_icp_localizer;
};