#pragma once

#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "file_writer.h"
#include "pgos/commons.h"

namespace utils {

class PointCloud3DTo2DGrid {
 public:
  void convert(CloudType::Ptr cloud, const std::string& output_dir, const std::string& prefix,
               float z_min = -10000, float z_max = 10000, float resolution = 0.05);
};
}  // namespace utils