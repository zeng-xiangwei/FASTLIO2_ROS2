#pragma once
#include <iomanip>
#include <iostream>

#include <builtin_interfaces/msg/time.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

namespace robosense_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace robosense_ros

// namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point, (float, x, x)(float, y, y)(float, z, z)
                                  // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                  (float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp,
                                                                                           timestamp))

class Utils {
 public:
  // static pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::msg::PointCloud2 &msg);
  // static sensor_msgs::msg::PointCloud2 convertToROS(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  static double getSec(std_msgs::msg::Header& header);
  static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg,
                                                              int filter_num, double min_range = 0.5,
                                                              double max_range = 20.0, double min_z = -1000.0,
                                                              double max_z = 1000.0);
  static pcl::PointCloud<pcl::PointXYZINormal>::Ptr robosense2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                                                  int filter_num, double min_range = 0.5,
                                                                  double max_range = 20.0, int n_scans = 96);
  static builtin_interfaces::msg::Time getTime(const double& sec);
};