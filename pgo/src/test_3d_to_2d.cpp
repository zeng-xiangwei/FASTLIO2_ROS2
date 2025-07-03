#include <pcl/io/pcd_io.h>

#include "utils/point_cloud_3d_to_2d_grid.h"

int main(int argc, char** argv) {
  std::cout << "argc: " << argc << std::endl;
  for (int i = 0; i < argc; i++) {
    std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
  }
  if (argc != 7) {
    std::cout << "need element: pcd_path, output_dir, prefix, z_min, z_max, resolution" << std::endl;
    return 0;
  }
  CloudType::Ptr cloud_temp(new CloudType());
  std::string pcd_path = argv[1];
  pcl::PCDReader reader;
  reader.read(pcd_path, *cloud_temp);

  utils::PointCloud3DTo2DGrid point_cloud_3d_to_2d_grid;

  std::string dir = argv[2];
  std::string prefix = argv[3];
  float z_min = std::atof(argv[4]);
  float z_max = std::atof(argv[5]);
  float resolution = std::atof(argv[6]);
  std::cout << "z_min: " << z_min << ", z_max: " << z_max << ", resolution: " << resolution << std::endl;
  point_cloud_3d_to_2d_grid.convert(cloud_temp, dir, prefix, z_min, z_max, resolution);
}