#pragma once

#include "2d_grid_map/map_limit.h"
#include "pgos/commons.h"

namespace utils {

class OccupancyMap {
 public:
  struct Config {
    double resolution = 0.05;
    double max_z = 1.0;
    double min_z = -0.3;
    int occupancy_weight = 30;
  };

  OccupancyMap(const Config& config);

  /// 往这个占据栅格地图中增加一个frame
  void AddLidarFrame(CloudType::Ptr lidar_cloud, const Eigen::Vector3d& translation,
                     const Eigen::Quaterniond& rotation);

  void Save(const std::string& dir, const std::string& prefix);

 private:
  /// 在某个点填入占据或者非占据信息
  void SetPoint(const Eigen::Array2i& pt, bool occupy, bool allow_revisit = false, uint8_t delta = 1u);
  void GrowAsNeeded();
  void GrowLimits(const Eigen::Vector2f& point);

  uint8_t* GetMutableValue(const Eigen::Array2i& pt);
  uint8_t GetValue(const Eigen::Array2i& pt) const;
  bool Visited(const Eigen::Array2i& pt) const;
  void SetVisited(const Eigen::Array2i& pt);

  void ComputeCroppedLimits(Eigen::Array2i* const offset, CellLimits* const limits) const;
  std::pair<std::vector<uint8_t>, MapLimits> ComputeCroppedGrid();
  /**
   * Bresenham直线填充，给定起始点和终止点，将中间的区域填充为白色
   * @param p1
   * @param p2
   */
  void BresenhamFilling(const Eigen::Array2i& p1, const Eigen::Array2i& p2);

  Config config_;
  std::vector<uint8_t> occupancy_grid_data_;
  std::vector<bool> occupancy_visited_;
  std::shared_ptr<MapLimits> map_limits_;
  Eigen::AlignedBox2i known_cells_box_;

  // 根据z轴筛选的点云数据
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> cloud_w_;
  Eigen::Vector3f t_w_l_;
  Eigen::Quaternionf r_w_l_;
};

}  // namespace utils
