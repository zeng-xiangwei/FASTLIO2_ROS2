#pragma once
#include "commons.h"
#include <stdint.h>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/voxel_grid.h>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_dis_thresh = 1.0;
    double refine_score_thresh = 0.1;
    double refine_max_correspondence_dis = 1.0;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_dis_thresh = 1.0;
    double rough_score_thresh = 0.2;
    double rough_max_correspondence_dis = 1.0;
    int rough_max_iteration = 5;
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);
    
    bool loadMap(const std::string &path);
    
    void setInput(const CloudType::Ptr &cloud);

    bool align(M4F &guess);
    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap() { return m_rough_tgt; }
    CloudType::Ptr refineMap() { return m_refine_tgt; }
    
    double getRoughScore() { return rough_score; }
    double getRefineScore() { return refine_score; }
private:
    ICPConfig m_config;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    pcl::IterativeClosestPoint<PointType, PointType> m_refine_icp;
    // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_refine_icp;
    pcl::IterativeClosestPoint<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
    double rough_score = 0.0;
    double refine_score = 0.0;
};