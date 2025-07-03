#include <glog/logging.h>

#include "lio_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto node = std::make_shared<LIONode>("lio_node");
  node->initRos();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}