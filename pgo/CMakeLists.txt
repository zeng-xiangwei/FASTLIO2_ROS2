cmake_minimum_required(VERSION 3.8)
project(pgo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(SRC_LIST src/pgos/commons.cpp
             src/pgos/simple_pgo.cpp
             src/utils/file_writer.cpp
             src/utils/point_cloud_3d_to_2d_grid.cpp
             src/utils/occupancy_map.cpp)

add_executable(pgo_node src/pgo_node.cpp ${SRC_LIST})
ament_target_dependencies(pgo_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs visualization_msgs interface GTSAM)
target_link_libraries(pgo_node ${PCL_LIBRARIES} gtsam yaml-cpp glog::glog)

set(tools_3d_to_2d_src_list
  src/tools_3d_to_2d.cpp
  src/utils/point_cloud_3d_to_2d_grid.cpp
  src/utils/file_writer.cpp
  src/utils/occupancy_map.cpp
)
add_executable(tools_3d_to_2d_gridmap ${tools_3d_to_2d_src_list})
target_link_libraries(tools_3d_to_2d_gridmap ${PCL_LIBRARIES} glog::glog yaml-cpp)
ament_target_dependencies(tools_3d_to_2d_gridmap rclcpp)


install(TARGETS pgo_node tools_3d_to_2d_gridmap DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
