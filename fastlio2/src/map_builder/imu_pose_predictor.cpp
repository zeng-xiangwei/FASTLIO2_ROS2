#include "imu_pose_predictor.h"

#include <glog/logging.h>

ImuPosePredictor::ImuPosePredictor() { latest_lio_state_.timestamp = -1; }

void ImuPosePredictor::setLioState(const StateWithTime& lio_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  CHECK_GT(lio_state.timestamp, latest_lio_state_.timestamp) << "LIO state timestamp is not monotonically increasing.";
  latest_lio_state_ = lio_state;

  latest_imu_state_ = latest_lio_state_;

  if (imu_data_queue_.empty()) {
    return;
  }

  double latest_lio_time = latest_lio_state_.timestamp;
  // drop all useless imu pkg
  while ((!imu_data_queue_.empty() && imu_data_queue_.front().time < latest_lio_time)) {
    imu_data_queue_.pop_front();
  }

  if (latest_imu_data_ == nullptr) {
    latest_imu_data_ = std::make_shared<IMUData>();
  }
  *latest_imu_data_ = imu_data_queue_.front();

  double predict_duration = 0;
  for (size_t i = 0; i < imu_data_queue_.size(); i++) {
    IMUData& imu_data = imu_data_queue_[i];
    double dt = imu_data.time - latest_imu_state_.timestamp;
    LOG_IF(WARNING, dt > 0.011) << "IMU delta time is larger than 0.011s(100hz)";

    V3D acc = (imu_data.acc + latest_imu_data_->acc) * 0.5;
    V3D gyro = (imu_data.gyro + latest_imu_data_->gyro) * 0.5;

    propogate(latest_imu_state_.state, dt, acc, gyro);
    latest_imu_state_.timestamp = imu_data.time;
    *latest_imu_data_ = imu_data;

    predict_duration += dt;
  }
  LOG(INFO) << "after set lio state, predict duration: " << predict_duration << " s";
}

void ImuPosePredictor::propogate(State& state, double dt, const V3D& acc_avr, const V3D& angvel_avr) {
  V21D delta = V21D::Zero();
  delta.segment<3>(0) = (angvel_avr - state.bg) * dt;

  // TODO: 位置更新用哪种？fast-livo2 中用的是 p = p + v*dt + 0.5*a*dt*dt 的方式
  // delta.segment<3>(3) = state.v * dt;
  delta.segment<3>(3) = state.v * dt + 0.5 * (state.r_wi * (acc_avr - state.ba)) * dt * dt + 0.5 * state.g * dt * dt;

  delta.segment<3>(12) = (state.r_wi * (acc_avr - state.ba) + state.g) * dt;

  state += delta;
}

void ImuPosePredictor::addImuData(const IMUData& imu_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!imu_data_queue_.empty() && imu_data_queue_.back().time > imu_data.time) {
    CHECK(false) << "IMU data is not in chronological order!";
  }
  imu_data_queue_.push_back(imu_data);
  LOG_IF_EVERY_N(WARNING, imu_data_queue_.size() > 1000, 500) << "Imu data queue is too long.";
  if (!validToPropogate()) {
    return;
  }

  if (latest_imu_data_ == nullptr) {
    latest_imu_data_ = std::make_shared<IMUData>(imu_data);
  }

  double dt = imu_data.time - latest_imu_state_.timestamp;
  LOG_IF(WARNING, dt > 0.011) << "IMU delta time is larger than 0.011s(100hz)";
  LOG_IF(WARNING, dt < 0.0) << "IMU delta time is smaller than 0.0s";

  V3D acc = (imu_data.acc + latest_imu_data_->acc) * 0.5;
  V3D gyro = (imu_data.gyro + latest_imu_data_->gyro) * 0.5;
  propogate(latest_imu_state_.state, dt, acc, gyro);
  latest_imu_state_.timestamp = imu_data.time;
  *latest_imu_data_ = imu_data;
}

bool ImuPosePredictor::validToPropogate() const {
  if (latest_lio_state_.timestamp > 0) {
    return true;
  }
  return false;
}

bool ImuPosePredictor::getPredictedState(StateWithTime& state_out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!validToPropogate()) {
    return false;
  }
  state_out = latest_imu_state_;
  return true;
}