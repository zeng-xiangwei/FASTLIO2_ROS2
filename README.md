# 新增功能
1. 采用替换 fastlio 中的全局地图来实现定位，定位稳定性会好一点
2. 定位模式下，在位姿变化大于阈值时，保存一次位姿。该位姿用于下一次重定位，提高效率，避免每次都是手动重定位
3. 增加一个位姿转换节点，用于发布车体系位姿，用于规控模块
4. 增加导出 2d 占据栅格地图功能，目前支持两种方式：直接将 3d 点云地图转为 2d 栅格，暂时仅支持占据和非占据；将关键帧+位姿持续更新 2d 栅格地图。


# FASTLIO2 ROS2
## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)

## 环境依赖
1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
glog
livox_ros_driver2
```

## 详细说明
### 1.编译 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.编译 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
# 如果 gcc 版本是 12，则需要添加 -DBUILD_SOPHUS_TESTS=OFF
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_SOPHUS_TESTS=OFF
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**

### 4. gtsam 4.2.0
```shell
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2.0
mkdir build
cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install
```

### 5. glog
```shell
sudo apt-get install libgflags-dev libgoogle-glog-dev
```

## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

### 5.直接替换 fast-lio 中的地图实现定位
```shell
ros2 launch fastlio2 localization.launch.py
```
使用 rviz2 中的 2D Pose Estimate 发布粗略的初始位姿，精确的初始位姿计算方法与重定位模块中的方法一致

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## 性能相关的问题
该代码主要使用timerCB作为频率触发主函数，由于ROS2中的timer、subscriber以及service的回调实际上运行在同一个线程上，在电脑性能不是好的时候，会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来(如timerCB)来提升性能

