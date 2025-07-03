#include "point_cloud_3d_to_2d_grid.h"

namespace utils {
void PointCloud3DTo2DGrid::convert(CloudType::Ptr cloud, const std::string& output_dir, const std::string& prefix,
                                   float z_min, float z_max, float resolution) {
  // 给定需要保留的z值范围[zmin, zmax]，取该范围内的点云，以及 xy范围
  CloudType cloud_filtered;
  float x_min = std::numeric_limits<float>::max();
  float x_max = std::numeric_limits<float>::lowest();
  float y_min = std::numeric_limits<float>::max();
  float y_max = std::numeric_limits<float>::lowest();

  for (size_t i = 0; i < cloud->points.size(); i++) {
    if (cloud->points[i].z >= z_min && cloud->points[i].z <= z_max) {
      cloud_filtered.push_back(cloud->points[i]);
      x_min = std::min(x_min, cloud->points[i].x);
      x_max = std::max(x_max, cloud->points[i].x);
      y_min = std::min(y_min, cloud->points[i].y);
      y_max = std::max(y_max, cloud->points[i].y);
    }
  }
  std::cout << "in [zmin, zmax] point size: " << cloud_filtered.points.size() << std::endl;
  std::cout << "x_min: " << x_min << ", x_max: " << x_max << ", y_min: " << y_min << ", y_max: " << y_max << std::endl;

  // 将提取的点云投影到栅格地图中, 按行写入到文件, 并导出pgm图片以及yaml参数
  // 以包围框的左上角为起点，resolution为像素长度，做一张图片
  // 需要计算一下这个图片的左下角的坐标，作为origin的值，不是直接拿包围框的左下角，因为不是一个完整的像素
  // kPaddingPixel的作用是让图片比包围框大
  const int kPaddingPixel = 5;
  int x_length = std::floor((x_max - x_min) / resolution) + kPaddingPixel;
  int y_length = std::floor((y_max - y_min) / resolution) + kPaddingPixel;
  char occupied_value = 0, free_value = 255;
  std::vector<std::vector<char>> grid_image_map(y_length, std::vector<char>(x_length, free_value));
  for (size_t i = 0; i < cloud_filtered.points.size(); i++) {
    int x_index = std::floor((cloud_filtered.points[i].x - x_min) / resolution);
    int y_index = std::floor((y_max - cloud_filtered.points[i].y) / resolution);
    grid_image_map[y_index][x_index] = occupied_value;
  }

  // 导出pgm
  std::string map_filestem = output_dir.back() == '/' ? output_dir + prefix : output_dir + "/" + prefix;
  StreamFileWriter pgm_writer(map_filestem + ".pgm");
  std::cout << "save pgm to " << pgm_writer.GetFilename() << std::endl;
  int image_width = grid_image_map[0].size();
  int image_height = grid_image_map.size();
  const std::string header = "P5\n# fast_lio map; " + std::to_string(resolution) + " m/pixel\n" +
                             std::to_string(image_width) + " " + std::to_string(image_height) + "\n255\n";
  pgm_writer.Write(header.data(), header.size());

  const char r_max = (char)255;
  const char unknown = (char)128;
  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      const char color = grid_image_map[y][x];
      pgm_writer.Write(&color, 1);
    }
  }

  // 因为是按照包围框的左上角画栅格图，因此左下角的x=x_min，y要取左下角的像素坐标（像素的左下角）
  const Eigen::Vector2d origin(x_min, y_max - y_length * resolution);

  // 导出yaml
  StreamFileWriter yaml_writer(map_filestem + ".yaml");
  std::cout << "save yaml to " << yaml_writer.GetFilename() << std::endl;
  std::string relative_pgm_path = prefix + ".pgm";
  const std::string output = "image: " + relative_pgm_path + "\n" + "resolution: " + std::to_string(resolution) + "\n" +
                             "origin: [" + std::to_string(origin.x()) + ", " + std::to_string(origin.y()) + ", 0.0]\n" +
                             "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196";
  yaml_writer.Write(output.data(), output.size());

  pcl::PCDWriter pcd_writer;
  std::string extracted_cloud_path = map_filestem + ".pcd";
  std::cout << "save extracted cloud to " << extracted_cloud_path << std::endl;
  pcd_writer.writeBinary(extracted_cloud_path, cloud_filtered);
}
}  // namespace utils