#ifndef IMU_POSE_PREDICTOR_H_
#define IMU_POSE_PREDICTOR_H_

#include <Eigen/Eigen>
#include <deque>
#include <mutex>

#include "commons.h"
#include "ieskf.h"

class ImuPosePredictor {
public:
    ImuPosePredictor();

    // 设置最新的 LIO 状态
    void setLioState(const StateWithTime& lio_state);

    // 添加 IMU 数据
    void addImuData(const IMUData& imu_data);

    // 获取当前预测的 IMU 状态
    bool getPredictedState(StateWithTime& output) const;
private:
    void propogate(State& state, double dt, const V3D& acc_avr, const V3D& angvel_avr);

    // 判断是否可以进行预测
    bool validToPropogate() const;

private:
    mutable std::mutex mutex_;
    StateWithTime latest_lio_state_;
    std::deque<IMUData> imu_data_queue_;
    double last_imu_predict_timestamp_ = -1;
    
    // 状态 与 数据需要绑定到一起，数据用于计算中值积分，latest_imu_data_中时间不使用
    StateWithTime latest_imu_state_;
    std::shared_ptr<IMUData> latest_imu_data_;
};

#endif // IMU_POSE_PREDICTOR_H_