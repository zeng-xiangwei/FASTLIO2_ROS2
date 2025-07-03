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
    CloudType::Ptr cloud_world;
    if (m_localization_global_map != nullptr) {
      cloud_world = m_localization_global_map;
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