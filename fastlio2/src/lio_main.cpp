#include <glog/logging.h>

#include "lio_node.h"
#include "pose_transform_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto node = std::make_shared<LIONode>("lio_node");
  node->initRos();
  
  auto pose_transform_node = std::make_shared<PoseTransformNode>("pose_transform_node");
  pose_transform_node->initRos();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), node->getNodeConfig().ros_spin_thread);
  executor.add_node(node);
  executor.add_node(pose_transform_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}