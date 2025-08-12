#include <fstream>
#include <iostream>
#include <sstream>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utils/file_writer.h"

namespace {
const uint8_t kUnknown = 128u;
const uint8_t kMaxPixelValue = 255u;
const uint8_t kMinPixelValue = 0u;
}  // namespace

class OccupancyMapSaver : public rclcpp::Node {
 public:
  OccupancyMapSaver() : Node("occupancy_map_saver") {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&OccupancyMapSaver::mapCallback, this, std::placeholders::_1));

    dir_ = this->declare_parameter("map_dir", "");
    prefix_ = this->declare_parameter("map_prefix", "");
    RCLCPP_INFO(this->get_logger(), "OccupancyMapSaver node started, waiting for map message...");
  }

 private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // 只处理第一次接收到的地图消息
    static bool received = false;
    if (received) {
      return;
    }
    received = true;

    RCLCPP_INFO(this->get_logger(), "Received map message, saving to file...");

    // 保存为PGM文件
    saveMapAsPGM(msg, dir_, prefix_);

    RCLCPP_INFO(this->get_logger(), "Map saved successfully, shutting down...");

    // 保存完成后结束程序
    rclcpp::shutdown();
  }

  void saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map, const std::string& output_dir,
                    const std::string& prefix) {
    // 导出pgm
    std::string map_filestem = output_dir.back() == '/' ? output_dir + prefix : output_dir + "/" + prefix;
    utils::StreamFileWriter pgm_writer(map_filestem + ".pgm");
    std::cout << "save pgm to " << pgm_writer.GetFilename() << std::endl;

    int image_width = map->info.width;
    int image_height = map->info.height;
    float resolution = map->info.resolution;
    const std::string header = "P5\n# map; " + std::to_string(resolution) + " m/pixel\n" + std::to_string(image_width) +
                               " " + std::to_string(image_height) + "\n255\n";
    pgm_writer.Write(header.data(), header.size());
    std::cout << "pgm header " << header << std::endl;

    std::cout << "occupancy_data size: " << map->data.size() << std::endl;
    for (int y = image_height - 1; y >= 0; --y) {
      for (int x = 0; x < image_width; ++x) {
        int8_t occupancy_data = map->data[x + y * image_width];
        uint8_t value = kUnknown;
        if (occupancy_data >= 0 && occupancy_data <= 100) {
          value = (100 - occupancy_data) / 100.0 * 255.0;
        }

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
    }

    // 因为是按照包围框的左上角画栅格图，因此左下角的x=x_min，y要取左下角的像素坐标（像素的左下角）
    float origin_x = map->info.origin.position.x;
    float origin_y = map->info.origin.position.y;

    // 导出yaml
    utils::StreamFileWriter yaml_writer(map_filestem + ".yaml");
    std::cout << "save yaml to " << yaml_writer.GetFilename() << std::endl;
    std::string relative_pgm_path = prefix + ".pgm";
    const std::string output = "image: " + relative_pgm_path + "\n" + "resolution: " + std::to_string(resolution) +
                               "\n" + "origin: [" + std::to_string(origin_x) + ", " + std::to_string(origin_y) +
                               ", 0.0]\n" + "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196";
    yaml_writer.Write(output.data(), output.size());
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  std::string dir_;
  std::string prefix_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyMapSaver>());
  rclcpp::shutdown();
  return 0;
}