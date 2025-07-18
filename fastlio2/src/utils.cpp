#include "utils.h"

#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg,
                                                            int filter_num, double min_range, double max_range) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
  int point_num = msg->point_num;
  cloud->reserve(point_num / filter_num + 1);
  for (int i = 0; i < point_num; i += filter_num) {
    if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
      float x = msg->points[i].x;
      float y = msg->points[i].y;
      float z = msg->points[i].z;
      if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range) continue;
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
  pcl::PointCloud<robosense_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

  if (pl_orig.points.empty()) {
    return cloud;
  }

  int point_num = pl_orig.points.size();
  cloud->reserve(point_num / filter_num + 1);
  double first_point_time = pl_orig.points[0].timestamp;

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
      float x = pl_orig.points[idx].x;
      float y = pl_orig.points[idx].y;
      float z = pl_orig.points[idx].z;
      if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range) continue;
      pcl::PointXYZINormal p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = pl_orig.points[idx].intensity;
      float relative_time = pl_orig.points[idx].timestamp - first_point_time;
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
