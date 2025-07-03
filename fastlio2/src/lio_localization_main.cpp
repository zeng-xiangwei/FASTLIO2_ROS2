#include <glog/logging.h>

#include "lio_localization_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto node = std::make_shared<LIOLocalizationNode>("localization_node");
  node->initRos();
  
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), node->getNodeConfig().ros_spin_thread);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}