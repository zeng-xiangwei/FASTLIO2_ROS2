#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "utils/occupancy_map.h"
#include "utils/point_cloud_3d_to_2d_grid.h"

namespace grid_map_utils {
class GridMapUtils {
 public:
  enum OccType {
    // 一帧一帧拼接，按照占据栅格的方式构建（黑白灰）
    BLACK_WHITE_GRAY = 0,
    // 直接将全局地图构建栅格地图（仅黑白）
    BLACK_WHITE = 1
  };

  struct GridMapConfig {
    // 保存的关键帧点云文件夹路径
    std::string patches_path_dir;
    // 关键帧位姿路径
    std::string pose_file;
    // 合并后的点云地图文件路径
    std::string combined_pcd_file;
    // 栅格地图输出文件路径
    std::string output_dir;
    // 栅格地图文件名前缀
    std::string name_prefix;

    float z_min;
    float z_max;
    float resolution;
    int occupancy_weight;

    OccType occupancy_map_type = BLACK_WHITE_GRAY;
  };

  struct Pose {
    Eigen::Quaterniond r;
    V3D t;
  };

  void readConfig(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    if (!config) {
      std::cout << "cannot load config file\n";
      return;
    }
    config_.patches_path_dir = config["patches_path_dir"].as<std::string>();
    config_.pose_file = config["pose_file"].as<std::string>();
    config_.output_dir = config["output_dir"].as<std::string>();
    config_.name_prefix = config["name_prefix"].as<std::string>();
    config_.combined_pcd_file = config["combined_pcd_file"].as<std::string>();
    config_.z_min = config["z_min"].as<float>();
    config_.z_max = config["z_max"].as<float>();
    config_.resolution = config["resolution"].as<float>();
    config_.occupancy_weight = config["occupancy_weight"].as<int>();
    
    std::string occ_type = config["occupancy_type"].as<std::string>();
    if (occ_type == "BLACK_WHITE_GRAY") {
      config_.occupancy_map_type = BLACK_WHITE_GRAY;
    } else {
      config_.occupancy_map_type = BLACK_WHITE;
    }

    std::cout << "config loaded\n";
    std::cout << "combined_pcd_file: " << config_.combined_pcd_file << "\n";
    std::cout << "patches_path_dir: " << config_.patches_path_dir << "\n";
    std::cout << "pose_file: " << config_.pose_file << "\n";
    std::cout << "output_dir: " << config_.output_dir << "\n";
    std::cout << "name_prefix: " << config_.name_prefix << "\n";
    std::cout << "z_min: " << config_.z_min << "\n";
    std::cout << "z_max: " << config_.z_max << "\n";
    std::cout << "resolution: " << config_.resolution << "\n";
  }

  void run() {
    if (config_.occupancy_map_type == BLACK_WHITE_GRAY) {
      constructOccupancyMap();
    } else {
      wholeMap2GridMap();
    }
  }

private:

  void wholeMap2GridMap() {
    pcl::PCDReader reader;
    CloudType::Ptr cloud_temp(new CloudType());
    reader.read(config_.combined_pcd_file, *cloud_temp);

    utils::PointCloud3DTo2DGrid point_cloud_3d_to_2d_grid;
    point_cloud_3d_to_2d_grid.convert(cloud_temp, config_.output_dir, config_.name_prefix, config_.z_min, config_.z_max,
                                      config_.resolution);
  }

  void fromStr(const std::string& str, std::string& file_name, Pose& pose) {
    std::stringstream ss(str);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ' ')) {
      tokens.push_back(token);
    }
    assert(tokens.size() == 8);
    file_name = tokens[0];
    pose.t = V3D(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
    pose.r = Eigen::Quaterniond(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]));
  }

  void constructOccupancyMap() {
    utils::OccupancyMap::Config config;
    config.resolution = config_.resolution;
    config.min_z = config_.z_min;
    config.max_z = config_.z_max;
    config.occupancy_weight = config_.occupancy_weight;
    utils::OccupancyMap occupancy_map(config);

    // 读取关键帧点云+位姿
    if (!std::filesystem::exists(config_.patches_path_dir)) {
      std::cout << "patches_path_dir not exists\n";
      return;
    }

    std::filesystem::path pcd_dir(config_.patches_path_dir);
    std::filesystem::path txt_file = config_.pose_file;

    if (!std::filesystem::exists(txt_file)) {
      std::cout << "pose_file not exists\n";
      return;
    }

    std::ifstream ifs(txt_file);
    std::string line;
    std::string file_name;
    Pose pose;
    pcl::PCDReader reader;
    while (std::getline(ifs, line)) {
      fromStr(line, file_name, pose);
      std::filesystem::path pcd_file = pcd_dir / file_name;
      if (!std::filesystem::exists(pcd_file)) {
        std::cerr << "pcd file not found" << std::endl;
        continue;
      }
      CloudType::Ptr cloud(new CloudType());
      reader.read(pcd_file, *cloud);

      occupancy_map.AddLidarFrame(cloud, pose.t, pose.r);
    }

    occupancy_map.Save(config_.output_dir, config_.name_prefix);
  }

 private:
  GridMapConfig config_;
};

}  // namespace grid_map_utils

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grid_map_tool");
  node->declare_parameter("config_file", "");
  std::string config_file;
  node->get_parameter("config_file", config_file);

  std::cout << "config_file: " << config_file << std::endl;
  grid_map_utils::GridMapUtils grid_map_tool;
  grid_map_tool.readConfig(config_file);
  grid_map_tool.run();
}