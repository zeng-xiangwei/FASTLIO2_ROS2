#include <glog/logging.h>
#include "lio_node.h"

#include "accumulate_lidar_cloud_for_calib.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LIONode>("lio_node");
  node->initRos();
  
  auto accumulate_lidar_cloud_node = std::make_shared<AccumulateLidarCloudForCalibNode>("accumulate_lidar_cloud");
  accumulate_lidar_cloud_node->initRos();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), node->getNodeConfig().ros_spin_thread);
  executor.add_node(node);
  executor.add_node(accumulate_lidar_cloud_node);
  executor.spin();

  accumulate_lidar_cloud_node->save();
  
  rclcpp::shutdown();
  return 0;
}