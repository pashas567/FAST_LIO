#pragma once

#include <Eigen/Eigenvalues>
#include "fast_lio/msg/filter_state.hpp"
#include "imu_prediction_core.hpp"

namespace fastlio_prediction {

inline int64_t stampNs(const builtin_interfaces::msg::Time & stamp) {
  return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}

inline builtin_interfaces::msg::Time rosStamp(int64_t stamp_ns) {
  builtin_interfaces::msg::Time stamp;
  int64_t seconds = stamp_ns / 1000000000LL;
  int64_t remainder = stamp_ns % 1000000000LL;
  if (remainder < 0) {--seconds; remainder += 1000000000LL;}
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>(remainder);
  return stamp;
}

inline geometry_msgs::msg::Quaternion quaternionMessage(const SO3 & q) {
  geometry_msgs::msg::Quaternion out;
  out.w = q.w(); out.x = q.x(); out.y = q.y(); out.z = q.z();
  return out;
}

template<typename VectorMessage>
inline VectorMessage vectorMessage(const Eigen::Vector3d & vector) {
  VectorMessage out;
  out.x = vector.x(); out.y = vector.y(); out.z = vector.z();
  return out;
}

template<typename VectorMessage>
inline Eigen::Vector3d eigenVector(const VectorMessage & vector) {
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

inline fast_lio::msg::FilterState encode(const Snapshot & s) {
  fast_lio::msg::FilterState msg;
  msg.schema_version = 1;
  msg.session_id = s.session_id;
  msg.epoch = s.epoch;
  msg.snapshot_sequence = s.sequence;
  msg.correction_sequence = s.correction_sequence;
  msg.header.stamp = rosStamp(s.stamp_ns);
  msg.header.frame_id = "camera_init";
  msg.last_correction_stamp_ns = s.correction_stamp_ns;
  msg.release_stamp_ns = s.release_stamp_ns;
  msg.phase = s.phase;
  msg.initialized = s.initialized;
  msg.recovery_requires_restart = s.recovery_requires_restart;
  msg.position = vectorMessage<geometry_msgs::msg::Point>(s.state.pos);
  msg.orientation = quaternionMessage(s.state.rot);
  msg.velocity_world = vectorMessage<geometry_msgs::msg::Vector3>(s.state.vel);
  msg.gyro_bias = vectorMessage<geometry_msgs::msg::Vector3>(s.state.bg);
  msg.accel_bias = vectorMessage<geometry_msgs::msg::Vector3>(s.state.ba);
  msg.gravity = vectorMessage<geometry_msgs::msg::Vector3>(s.state.grav.get_vect());
  msg.lidar_to_imu_rotation = quaternionMessage(s.state.offset_R_L_I);
  msg.lidar_to_imu_translation = vectorMessage<geometry_msgs::msg::Vector3>(s.state.offset_T_L_I);
  for (int row = 0; row < 23; ++row) {
    for (int col = 0; col < 23; ++col) {msg.covariance[row * 23 + col] = s.covariance(row, col);}
  }
  for (int row = 0; row < 12; ++row) {
    for (int col = 0; col < 12; ++col) {msg.process_noise[row * 12 + col] = s.noise(row, col);}
  }
  msg.acceleration_scale = s.acceleration_scale;
  msg.imu_time_offset_ns = s.imu_time_offset_ns;
  return msg;
}

inline bool decode(const fast_lio::msg::FilterState & msg, Snapshot & s, std::string & error) {
  if (msg.schema_version != 1 || msg.session_id.empty() || msg.header.frame_id != "camera_init" ||
      (msg.phase != "WAIT_INIT" && msg.phase != "GROUND_HOLD" &&
       msg.phase != "RELEASED_IMU_ONLY" && msg.phase != "LIDAR_TRACKING")) {
    error = "INVALID_SNAPSHOT_SCHEMA"; return false;
  }
  s.stamp_ns = stampNs(msg.header.stamp);
  s.session_id = msg.session_id;
  s.epoch = msg.epoch;
  s.sequence = msg.snapshot_sequence;
  s.correction_sequence = msg.correction_sequence;
  s.correction_stamp_ns = msg.last_correction_stamp_ns;
  s.release_stamp_ns = msg.release_stamp_ns;
  s.imu_time_offset_ns = msg.imu_time_offset_ns;
  s.phase = msg.phase;
  s.initialized = msg.initialized;
  s.recovery_requires_restart = msg.recovery_requires_restart;
  if (!s.initialized) {return true;}
  const auto & qmsg = msg.orientation;
  const auto & emsg = msg.lidar_to_imu_rotation;
  Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
  Eigen::Quaterniond e(emsg.w, emsg.x, emsg.y, emsg.z);
  s.state.pos = eigenVector(msg.position);
  s.state.vel = eigenVector(msg.velocity_world);
  s.state.bg = eigenVector(msg.gyro_bias);
  s.state.ba = eigenVector(msg.accel_bias);
  const Eigen::Vector3d gravity = eigenVector(msg.gravity);
  s.state.offset_T_L_I = eigenVector(msg.lidar_to_imu_translation);
  s.acceleration_scale = msg.acceleration_scale;
  if (!s.state.pos.allFinite() || !s.state.vel.allFinite() || !s.state.bg.allFinite() ||
      !s.state.ba.allFinite() || !gravity.allFinite() || gravity.norm() < 1. ||
      !s.state.offset_T_L_I.allFinite() || !q.coeffs().allFinite() || !e.coeffs().allFinite() ||
      std::abs(q.norm() - 1.) > 1e-6 || std::abs(e.norm() - 1.) > 1e-6 ||
      !std::isfinite(s.acceleration_scale) || s.acceleration_scale <= 0. ||
      s.stamp_ns <= 0 || s.correction_stamp_ns > s.stamp_ns) {
    error = "INVALID_SNAPSHOT_STATE"; return false;
  }
  s.state.rot = SO3(q.normalized());
  s.state.offset_R_L_I = SO3(e.normalized());
  s.state.grav = S2(gravity);
  for (int row = 0; row < 23; ++row) {
    for (int col = 0; col < 23; ++col) {s.covariance(row, col) = msg.covariance[row * 23 + col];}
  }
  for (int row = 0; row < 12; ++row) {
    for (int col = 0; col < 12; ++col) {s.noise(row, col) = msg.process_noise[row * 12 + col];}
  }
  if (!s.covariance.allFinite() || !s.noise.allFinite() ||
      !s.covariance.isApprox(s.covariance.transpose(), 1e-7) ||
      !s.noise.isApprox(s.noise.transpose(), 1e-7)) {
    error = "INVALID_SNAPSHOT_COVARIANCE"; return false;
  }
  Eigen::SelfAdjointEigenSolver<Covariance> eigen_p(s.covariance, Eigen::EigenvaluesOnly);
  Eigen::SelfAdjointEigenSolver<ProcessNoise> eigen_q(s.noise, Eigen::EigenvaluesOnly);
  if (eigen_p.info() != Eigen::Success || eigen_q.info() != Eigen::Success ||
      eigen_p.eigenvalues().minCoeff() < -1e-8 || eigen_q.eigenvalues().minCoeff() < -1e-10) {
    error = "NON_PSD_SNAPSHOT_COVARIANCE"; return false;
  }
  return true;
}

}  // namespace fastlio_prediction
