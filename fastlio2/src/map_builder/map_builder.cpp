#include "map_builder.h"

#include <glog/logging.h>
MapBuilder::MapBuilder(Config& config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf) {
  m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
  m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
  m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::process(SyncPackage& package) {
  if (m_status == BuilderStatus::IMU_INIT) {
    if (m_imu_processor->initialize(package)) {
      m_status = BuilderStatus::MAP_INIT;
    }
    return;
  }

  m_imu_processor->undistort(package);

  if (m_status == BuilderStatus::MAP_INIT) {
    CloudType::Ptr cloud_world = std::make_shared<CloudType>();
    if (m_localization_global_map != nullptr) {
      pcl::VoxelGrid<PointType> voxel_filter;
      voxel_filter.setLeafSize(m_config.map_resolution, m_config.map_resolution, m_config.map_resolution);
      voxel_filter.setInputCloud(m_localization_global_map);
      voxel_filter.filter(*cloud_world);
      LOG(INFO) << "Use localization global map, map size: " << cloud_world->points.size();
    } else {
      cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
    }
    m_lidar_processor->initCloudMap(cloud_world->points);
    m_status = BuilderStatus::MAPPING;
    return;
  }

  m_lidar_processor->process(package);
}