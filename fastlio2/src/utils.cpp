#include "utils.h"

#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg,
                                                            int filter_num, double min_range, double max_range,
                                                            double min_z, double max_z,
                                                            const Eigen::Vector3f& filter_box_min,
                                                            const Eigen::Vector3f& filter_box_max) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
  int point_num = msg->point_num;
  cloud->reserve(point_num / filter_num + 1);

  bool filter_by_box = (filter_box_max.array() > filter_box_min.array()).all();
  for (int i = 0; i < point_num; i += filter_num) {
    if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
      float x = msg->points[i].x;
      float y = msg->points[i].y;
      float z = msg->points[i].z;
      if (z < min_z || z > max_z) {
        continue;
      }
      if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range) {
        continue;
      }

      if (filter_by_box) {
        // 过滤掉 box 内的点云，主要用于过滤上身的机械臂
        if (x > filter_box_min.x() && x < filter_box_max.x() && y > filter_box_min.y() && y < filter_box_max.y() &&
            z > filter_box_min.z() && z < filter_box_max.z()) {
          continue;
        }
      }
      pcl::PointXYZINormal p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = msg->points[i].reflectivity;
      p.curvature = msg->points[i].offset_time / 1000000.0f;
      cloud->push_back(p);
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::robosense2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                                                int filter_num, double min_range, double max_range,
                                                                int n_scans) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

  int point_num = msg->width * msg->height;
  if (point_num == 0) {
    return cloud;
  }

  cloud->reserve(point_num / filter_num + 1);
  const uint8_t* first_point = &msg->data[0];
  double first_point_time = *(reinterpret_cast<const double*>(first_point + msg->fields[5].offset));
  std::cout << "first_point_time: " << first_point_time << std::endl;

  // 考虑到 robosense airy 的激光扫描是按列扫描，如果直接按filter_num降采样，会导致固定ring号的点云被降采样掉，
  // 为了使得每个 ring 内的点云被降采样，因此要按列跳过
  // 这种降采样方式要求驱动必须发布全量点云（包含 nan 点）
  int cols = point_num / n_scans;
  if (point_num % n_scans != 0) {
    // 保证 cols * n_scans <= point_num
    cols = cols - 1;
  }
  for (int ring_idx = 0; ring_idx < n_scans; ++ring_idx) {
    for (int col_idx = 0; col_idx < cols; col_idx += filter_num) {
      int idx = col_idx * n_scans + ring_idx;
      const uint8_t* point = &msg->data[idx * msg->point_step];
      float x = *(reinterpret_cast<const float*>(point + msg->fields[0].offset));
      float y = *(reinterpret_cast<const float*>(point + msg->fields[1].offset));
      float z = *(reinterpret_cast<const float*>(point + msg->fields[2].offset));
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        continue;
      }

      if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range) continue;
      pcl::PointXYZINormal p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = *(reinterpret_cast<const float*>(point + msg->fields[3].offset));
      double point_time = *(reinterpret_cast<const double*>(point + msg->fields[5].offset));
      float relative_time = point_time - first_point_time;
      p.curvature = relative_time * 1000.0f;
      cloud->push_back(p);
    }
  }

  return cloud;
}

double Utils::getSec(std_msgs::msg::Header& header) {
  return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double& sec) {
  builtin_interfaces::msg::Time time_msg;
  time_msg.sec = static_cast<int32_t>(sec);
  time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
  return time_msg;
}
