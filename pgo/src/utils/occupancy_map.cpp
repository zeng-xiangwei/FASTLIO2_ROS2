#include "occupancy_map.h"

#include <glog/logging.h>

#include "file_writer.h"

namespace utils {

const uint8_t kUnknown = 128u;
const uint8_t kOccupiedLowerBound = 118u;
const uint8_t kFreeUpperBound = 138u;
const uint8_t kMaxPixelValue = 255u;
const uint8_t kMinPixelValue = 0u;

OccupancyMap::OccupancyMap(const Config& config) : config_(config) { cloud_w_.reserve(2000); }

void OccupancyMap::AddLidarFrame(CloudType::Ptr lidar_cloud, const Eigen::Vector3d& translation,
                                 const Eigen::Quaterniond& rotation) {
  cloud_w_.clear();
  t_w_l_ = translation.cast<float>();
  r_w_l_ = rotation.cast<float>();
  r_w_l_.normalize();
  for (size_t i = 0; i < lidar_cloud->size(); ++i) {
    Eigen::Vector3f p_lidar(lidar_cloud->points[i].x, lidar_cloud->points[i].y, lidar_cloud->points[i].z);
    Eigen::Vector3f p_w = r_w_l_ * p_lidar + t_w_l_;
    if (p_w.z() < config_.min_z || p_w.z() > config_.max_z) {
      continue;
    }
    cloud_w_.push_back(p_w);
  }

  if (map_limits_ == nullptr) {
    map_limits_ =
        std::make_unique<MapLimits>(config_.resolution, Eigen::Vector2d(t_w_l_.x(), t_w_l_.y()), CellLimits(100, 100));
    occupancy_grid_data_ =
        std::vector<uint8_t>(map_limits_->cell_limits().num_x_cells * map_limits_->cell_limits().num_y_cells, kUnknown);
  }

  GrowAsNeeded();
  // 每次更新时，格子只更新一次
  occupancy_visited_ =
      std::vector<bool>(map_limits_->cell_limits().num_x_cells * map_limits_->cell_limits().num_y_cells, false);

  // 先计算末端点所在的网格
  std::vector<Eigen::Array2i> endpoints;

  for (size_t i = 0; i < cloud_w_.size(); ++i) {
    endpoints.push_back(map_limits_->GetCellIndex(cloud_w_[i].head<2>()));
  }

  Eigen::Array2i start = map_limits_->GetCellIndex({t_w_l_.x(), t_w_l_.y()});
  std::for_each(endpoints.begin(), endpoints.end(), [this, &start](const auto& pt) { BresenhamFilling(start, pt); });

  /// 末端点涂黑
  std::for_each(endpoints.begin(), endpoints.end(),
                [this](const auto& pt) { SetPoint(pt, true, config_.occupancy_weight); });
}

void OccupancyMap::SetPoint(const Eigen::Array2i& pt, bool occupy, uint8_t delta) {
  if (!map_limits_->Contains(pt)) {
    return;
  }

  if (Visited(pt)) {
    return;
  }
  SetVisited(pt);
  /// 这里设置了一个上下限
  uint8_t* value = GetMutableValue(pt);
  if (occupy) {
    if (*value > kOccupiedLowerBound) {
      *value -= delta;
    }
  } else {
    if (*value < kFreeUpperBound) {
      *value += delta;
    }
  }

  known_cells_box_.extend(pt.matrix());
}

void OccupancyMap::BresenhamFilling(const Eigen::Array2i& p1, const Eigen::Array2i& p2) {
  int dx = p2.x() - p1.x();
  int dy = p2.y() - p1.y();
  int ux = dx > 0 ? 1 : -1;
  int uy = dy > 0 ? 1 : -1;

  dx = abs(dx);
  dy = abs(dy);
  int x = p1.x();
  int y = p1.y();

  if (dx > dy) {
    // 以x为增量
    int e = -dx;
    for (int i = 0; i < dx; ++i) {
      x += ux;
      e += 2 * dy;
      if (e >= 0) {
        y += uy;
        e -= 2 * dx;
      }

      if ((Eigen::Array2i(x, y) != p2).any()) {
        SetPoint(Eigen::Array2i(x, y), false);
      }
    }
  } else {
    int e = -dy;
    for (int i = 0; i < dy; ++i) {
      y += uy;
      e += 2 * dx;
      if (e >= 0) {
        x += ux;
        e -= 2 * dy;
      }
      if ((Eigen::Array2i(x, y) != p2).any()) {
        SetPoint(Eigen::Array2i(x, y), false);
      }
    }
  }
}

uint8_t* OccupancyMap::GetMutableValue(const Eigen::Array2i& pt) {
  return &occupancy_grid_data_[pt.y() * map_limits_->cell_limits().num_x_cells + pt.x()];
}
uint8_t OccupancyMap::GetValue(const Eigen::Array2i& pt) const {
  return occupancy_grid_data_[pt.y() * map_limits_->cell_limits().num_x_cells + pt.x()];
}

void OccupancyMap::GrowAsNeeded() {
  Eigen::AlignedBox2f bounding_box(t_w_l_.head<2>());
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  constexpr float kPadding = 1e-6f;
  for (const auto& hit : cloud_w_) {
    bounding_box.extend(hit.head<2>());
  }
  GrowLimits(bounding_box.min() - kPadding * Eigen::Vector2f::Ones());
  GrowLimits(bounding_box.max() + kPadding * Eigen::Vector2f::Ones());
}

void OccupancyMap::GrowLimits(const Eigen::Vector2f& point) {
  while (!map_limits_->Contains(map_limits_->GetCellIndex(point))) {
    LOG(INFO) << "Growing map limits, xmin_ymax:" << map_limits_->xmin_ymax().transpose()
              << ", cell_limits: num_x_cells=" << map_limits_->cell_limits().num_x_cells
              << ", num_y_cells=" << map_limits_->cell_limits().num_y_cells;
    const int x_offset = map_limits_->cell_limits().num_x_cells / 2;
    const int y_offset = map_limits_->cell_limits().num_y_cells / 2;
    const MapLimits new_limits(
        map_limits_->resolution(),
        map_limits_->xmin_ymax() + map_limits_->resolution() * Eigen::Vector2d(-x_offset, y_offset),
        CellLimits(2 * map_limits_->cell_limits().num_x_cells, 2 * map_limits_->cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells * new_limits.cell_limits().num_y_cells;

    std::vector<uint8_t> new_cells(new_size, kUnknown);
    for (int i = 0; i < map_limits_->cell_limits().num_y_cells; ++i) {
      for (int j = 0; j < map_limits_->cell_limits().num_x_cells; ++j) {
        new_cells[offset + j + i * stride] = occupancy_grid_data_[j + i * map_limits_->cell_limits().num_x_cells];
      }
    }
    occupancy_grid_data_ = new_cells;
    *map_limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

void OccupancyMap::Save(const std::string& output_dir, const std::string& prefix) {
  // 导出pgm
  std::string map_filestem = output_dir.back() == '/' ? output_dir + prefix : output_dir + "/" + prefix;
  StreamFileWriter pgm_writer(map_filestem + ".pgm");
  std::cout << "save pgm to " << pgm_writer.GetFilename() << std::endl;

  std::pair<std::vector<uint8_t>, MapLimits> data_and_info = ComputeCroppedGrid();
  MapLimits& limits = data_and_info.second;
  std::vector<uint8_t>& occupancy_data = data_and_info.first;
  int image_width = limits.cell_limits().num_x_cells;
  int image_height = limits.cell_limits().num_y_cells;
  float resolution = limits.resolution();
  const std::string header = "P5\n# fast_lio map; " + std::to_string(resolution) + " m/pixel\n" +
                             std::to_string(image_width) + " " + std::to_string(image_height) + "\n255\n";
  pgm_writer.Write(header.data(), header.size());
  std::cout << "pgm header " << header << std::endl;

  std::cout << "occupancy_data size: " << occupancy_data.size() << std::endl;
  for (size_t i = 0; i < occupancy_data.size(); ++i) {
    uint8_t value = occupancy_data[i];
    if (value == kUnknown) {
      const char color = kUnknown;
      pgm_writer.Write(&color, 1);
    } else if (value < kUnknown) {
      const char color = kMinPixelValue;
      pgm_writer.Write(&color, 1);
    } else if (value > kUnknown) {
      const char color = kMaxPixelValue;
      pgm_writer.Write(&color, 1);
    }
  }

  // 因为是按照包围框的左上角画栅格图，因此左下角的x=x_min，y要取左下角的像素坐标（像素的左下角）
  const Eigen::Vector2d origin(limits.xmin_ymax().x(),
                               limits.xmin_ymax().y() - limits.cell_limits().num_y_cells * resolution);

  // 导出yaml
  StreamFileWriter yaml_writer(map_filestem + ".yaml");
  std::cout << "save yaml to " << yaml_writer.GetFilename() << std::endl;
  std::string relative_pgm_path = prefix + ".pgm";
  const std::string output = "image: " + relative_pgm_path + "\n" + "resolution: " + std::to_string(resolution) + "\n" +
                             "origin: [" + std::to_string(origin.x()) + ", " + std::to_string(origin.y()) + ", 0.0]\n" +
                             "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196";
  yaml_writer.Write(output.data(), output.size());
}

void OccupancyMap::ComputeCroppedLimits(Eigen::Array2i* const offset, CellLimits* const limits) const {
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();
  *limits = CellLimits(known_cells_box_.sizes().x() + 1, known_cells_box_.sizes().y() + 1);
}
std::pair<std::vector<uint8_t>, MapLimits> OccupancyMap::ComputeCroppedGrid() {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = map_limits_->resolution();
  const Eigen::Vector2d xmin_ymax = map_limits_->xmin_ymax() - resolution * Eigen::Vector2d(-offset.x(), offset.y());
  MapLimits cropped_map_limits(resolution, xmin_ymax, cell_limits);
  std::vector<uint8_t> cropped_grid(cell_limits.num_x_cells * cell_limits.num_y_cells, kUnknown);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    int index = xy_index.x() + xy_index.y() * cell_limits.num_x_cells;
    cropped_grid[index] = GetValue(xy_index + offset);
  }

  return {cropped_grid, cropped_map_limits};
}

bool OccupancyMap::Visited(const Eigen::Array2i& pt) const {
  int stride = map_limits_->cell_limits().num_x_cells;
  return occupancy_visited_[pt.x() + pt.y() * stride];
}

void OccupancyMap::SetVisited(const Eigen::Array2i& pt) {
  int stride = map_limits_->cell_limits().num_x_cells;
  occupancy_visited_[pt.x() + pt.y() * stride] = true;
}

}  // namespace utils