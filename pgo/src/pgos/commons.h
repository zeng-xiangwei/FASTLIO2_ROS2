#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M4F = Eigen::Matrix4f;
using V4F = Eigen::Vector4f;

struct PoseWithTime {
    V3D t;
    M3D r;
    int32_t sec;
    uint32_t nsec;
    double second;
    void setTime(int32_t sec, uint32_t nsec);
    // double second() const;
};

struct CloudWithPose {
    CloudType::Ptr cloud;
    PoseWithTime pose;
};

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};

struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

namespace YAML {
    template<>
    struct convert<V3D> {
        static Node encode(const V3D& vec) {
            Node node;
            node.push_back(vec.x());
            node.push_back(vec.y());
            node.push_back(vec.z());
            return node;
        }

        static bool decode(const Node& node, V3D& vec) {
            if(!node.IsSequence() || node.size() != 3) {
                return false;
            }
            vec.x() = node[0].as<double>();
            vec.y() = node[1].as<double>();
            vec.z() = node[2].as<double>();
            return true;
        }
    };
}