#include <iostream>
#include <stdexcept>
#include "imu_prediction_state.hpp"

namespace fp = fastlio_prediction;

void check(bool condition, const char * message) {
  if (!condition) {throw std::runtime_error(message);}
}

fp::Snapshot snapshot(int64_t stamp = 1000000000LL) {
  fp::Snapshot s;
  s.state.grav = S2(Eigen::Vector3d(0., 0., -9.809));
  s.session_id = "test-session";
  s.epoch = 1;
  s.sequence = 1;
  s.correction_sequence = 1;
  s.stamp_ns = stamp;
  s.correction_stamp_ns = stamp;
  s.release_stamp_ns = stamp;
  s.phase = "LIDAR_TRACKING";
  s.initialized = true;
  s.covariance = fp::Covariance::Identity() * 0.001;
  s.noise = fp::ProcessNoise::Identity() * 0.0001;
  return s;
}

fp::ImuSample sample(int64_t stamp, double acceleration_scale = 1.) {
  fp::ImuSample imu;
  imu.stamp_ns = stamp;
  imu.acceleration.z() = 9.809 / acceleration_scale;
  return imu;
}

fp::Settings settings(int rate = 30) {
  fp::Settings s;
  s.rate_hz = rate;
  s.max_correction_age_ns = 2000000000LL;
  return s;
}

void testGrid() {
  for (int rate : {20, 30, 40}) {
    fp::SourceGrid grid(rate);
    const int64_t epoch = 1791558687000000000LL;
    grid.reset(epoch);
    for (int n = 1; n <= 100000; ++n) {
      const int64_t expected = epoch + int64_t(n / rate) * 1000000000LL +
        (int64_t(n % rate) * 1000000000LL + rate / 2) / rate;
      check(grid.next() == expected, "source grid drifted or rounded its period");
      grid.advance();
    }
    grid.reset(epoch);
    check(grid.skipThrough(epoch + 1000000000LL) == uint64_t(rate), "grid skip is off by one");
    check(grid.next() > epoch + 1000000000LL, "grid repeated skipped timestamp");
  }
  bool rejected = false;
  try {fp::SourceGrid invalid(50);} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "unsupported 50 Hz parameter accepted");
}

using Outputs = std::vector<fp::Output, Eigen::aligned_allocator<fp::Output>>;

Outputs trajectory(int rate, double scale = 1.) {
  fp::Predictor core(settings(rate));
  fp::Snapshot initial = snapshot();
  initial.acceleration_scale = scale;
  initial.state.vel = Eigen::Vector3d(1., -0.5, 0.);
  core.append(sample(initial.stamp_ns, scale), [](const fp::Output &) {});
  check(core.accept(initial), "initial snapshot rejected");
  Outputs outputs;
  for (int index = 1; index <= 300; ++index) {
    core.append(sample(initial.stamp_ns + index * 5000000LL, scale),
      [&outputs](const fp::Output & out) {outputs.push_back(out);});
  }
  check(outputs.size() == size_t(rate * 3 / 2), "source rate dropped or duplicated outputs");
  for (const auto & out : outputs) {
    const double dt = double(out.stamp_ns - initial.stamp_ns) * 1e-9;
    check((out.state.pos - initial.state.vel * dt).norm() < 1e-11,
      "constant velocity/gravity cancellation prediction is wrong");
    check((out.state.vel - initial.state.vel).norm() < 1e-11,
      "normalization or gravity was applied twice");
    check(out.covariance.allFinite(), "nonfinite propagated covariance");
    Eigen::SelfAdjointEigenSolver<fp::Covariance> eigen(out.covariance);
    check(eigen.eigenvalues().minCoeff() > -1e-10, "prediction covariance lost PSD");
  }
  return outputs;
}

void testRatesAndScale() {
  const Outputs rate20 = trajectory(20);
  const Outputs rate30 = trajectory(30);
  const Outputs rate40 = trajectory(40);
  const Outputs scaled = trajectory(30, 9.809);
  for (size_t i = 0; i < rate20.size(); ++i) {
    check(rate20[i].stamp_ns == rate40[2 * i + 1].stamp_ns, "common rate timestamps disagree");
    check((rate20[i].covariance - rate40[2 * i + 1].covariance).norm() < 1e-12,
      "output frequency changed persistent covariance integration");
  }
  for (size_t i = 0; i < rate30.size(); ++i) {
    check((rate30[i].state.pos - scaled[i].state.pos).norm() < 1e-11,
      "configured acceleration scale differs from SI-equivalent prediction");
  }
}

void testDelayedAndFutureCorrection() {
  fp::Predictor core(settings());
  fp::Snapshot first = snapshot();
  first.state.vel.x() = 1.;
  Outputs out;
  const auto publish = [&out](const fp::Output & value) {out.push_back(value);};
  core.append(sample(first.stamp_ns), publish);
  core.accept(first);
  for (int i = 1; i <= 200; ++i) {core.append(sample(first.stamp_ns + i * 5000000LL), publish);}
  const int64_t previous = out.back().stamp_ns;
  fp::Snapshot delayed = first;
  delayed.stamp_ns += 800000000LL;
  delayed.correction_stamp_ns = delayed.stamp_ns;
  delayed.sequence = 2;
  delayed.correction_sequence = 2;
  delayed.state.pos.x() = 10.;
  delayed.state.vel.x() = 1.5;
  const size_t before = out.size();
  check(core.accept(delayed), "delayed correction rejected");
  check(out.size() == before, "correction replay republished old timestamps");
  for (int i = 201; i <= 207; ++i) {core.append(sample(first.stamp_ns + i * 5000000LL), publish);}
  check(out.size() == before + 1, "replay did not resume at next future grid point");
  check(out.back().stamp_ns > previous, "late correction regressed output time");
  const double elapsed = double(out.back().stamp_ns - delayed.stamp_ns) * 1e-9;
  check(std::abs(out.back().state.pos.x() - (10. + 1.5 * elapsed)) < 1e-10,
    "late correction did not replay from corrected position/velocity");
  check(!core.accept(delayed), "duplicate correction accepted");

  fp::Snapshot future = delayed;
  future.sequence = 3;
  future.correction_sequence = 3;
  future.stamp_ns = 2200000000LL;
  future.correction_stamp_ns = future.stamp_ns;
  future.state.pos.x() = 20.;
  check(core.accept(future), "future-delivered correction rejected");
  const size_t before_future = out.size();
  for (int i = 208; i <= 247; ++i) {core.append(sample(first.stamp_ns + i * 5000000LL), publish);}
  check(out.size() >= before_future + 5, "future correction discarded intervening source-time outputs");
  check(out.back().state.pos.x() > 20., "queued correction was never applied");
  bool found_boundary = false;
  for (const auto & value : out) {
    if (value.stamp_ns == future.stamp_ns) {
      found_boundary = true;
      check(std::abs(value.state.pos.x() - 20.) < 1e-10,
        "available correction was applied after its boundary output");
    }
  }
  check(found_boundary, "correction-boundary grid point was dropped");
  fp::Snapshot mid_bracket = future;
  mid_bracket.sequence = 4;
  mid_bracket.correction_sequence = 4;
  mid_bracket.stamp_ns = 2264000000LL;
  mid_bracket.correction_stamp_ns = mid_bracket.stamp_ns;
  mid_bracket.state.pos.x() = 30.;
  core.accept(mid_bracket);
  for (int i = 248; i <= 254; ++i) {core.append(sample(first.stamp_ns + i * 5000000LL), publish);}
  check(out.back().stamp_ns == 2266666667LL && out.back().state.pos.x() > 30.,
    "grid output inside new IMU bracket used the superseded posterior");
  for (size_t i = 1; i < out.size(); ++i) {
    check(out[i].stamp_ns > out[i - 1].stamp_ns, "correction output timestamps not strictly increasing");
  }
}

void testGapTimeoutEpochAndHold() {
  fp::Settings config;
  fp::Predictor core(config);
  fp::Snapshot initial = snapshot();
  Outputs outputs;
  const auto publish = [&outputs](const fp::Output & value) {outputs.push_back(value);};
  core.append(sample(initial.stamp_ns), publish);
  core.accept(initial);
  for (int i = 1; i <= 120; ++i) {core.append(sample(initial.stamp_ns + i * 5000000LL), publish);}
  check(!core.validAt(1600000000LL), "stale correction remained valid");
  check(outputs.back().stamp_ns <= 1500000000LL, "output extrapolated beyond correction horizon");
  fp::Snapshot refreshed = initial;
  refreshed.stamp_ns = 1600000000LL;
  refreshed.correction_stamp_ns = refreshed.stamp_ns;
  refreshed.sequence = 2;
  refreshed.correction_sequence = 2;
  core.accept(refreshed);
  core.append(sample(1605000000LL), publish);
  core.append(sample(1640000000LL), publish);
  check(!core.active() && core.reason() == "IMU_GAP", "long IMU gap was bridged");
  const auto rejected = core.rejectedImu();
  core.append(sample(1640000000LL), publish);
  core.append(sample(1635000000LL), publish);
  check(core.rejectedImu() == rejected + 2, "duplicate/out-of-order IMU accepted");
  fp::Snapshot rewind = initial;
  rewind.epoch = 2;
  rewind.sequence = 3;
  rewind.initialized = false;
  rewind.recovery_requires_restart = true;
  rewind.phase = "WAIT_INIT";
  rewind.stamp_ns = 0;
  core.accept(rewind);
  check(!core.accept(initial), "delayed snapshot revived a retired epoch");
  const auto stopped = outputs.size();
  core.append(sample(10000000LL), publish);
  core.append(sample(15000000LL), publish);
  check(outputs.size() == stopped && !core.active(), "rewound epoch reused old state");
  fp::Snapshot restarted = initial;
  restarted.session_id = "new-mapper-session";
  restarted.phase = "WAIT_INIT";
  restarted.initialized = false;
  core.accept(restarted);
  check(!core.accept(rewind), "delayed DDS snapshot revived a retired mapper session");

  fp::Predictor hold(settings());
  fp::Snapshot ground = snapshot();
  ground.phase = "GROUND_HOLD";
  ground.correction_sequence = 0;
  ground.correction_stamp_ns = 0;
  ground.release_stamp_ns = 0;
  hold.append(sample(ground.stamp_ns), publish);
  hold.accept(ground);
  Outputs held;
  for (int i = 1; i <= 40; ++i) {
    fp::ImuSample vibrating = sample(ground.stamp_ns + i * 5000000LL);
    vibrating.acceleration.x() = 1.; vibrating.gyro.z() = 0.1;
    hold.append(vibrating, [&held](const fp::Output & value) {held.push_back(value);});
  }
  check(!held.empty(), "ground-hold source stream missing");
  check(held.back().state.pos.norm() == 0. && held.back().state.vel.norm() == 0.,
    "ground hold integrated contact vibration");
  fp::Snapshot release = ground;
  release.sequence = 2;
  release.stamp_ns = 1200000000LL;
  release.release_stamp_ns = release.stamp_ns;
  release.phase = "RELEASED_IMU_ONLY";
  hold.accept(release);
  for (int i = 41; i <= 48; ++i) {
    auto moving = sample(ground.stamp_ns + i * 5000000LL);
    moving.acceleration.x() = 1.;
    hold.append(moving, [&held](const fp::Output & value) {held.push_back(value);});
  }
  check(held.back().state.vel.x() > 0.02, "release kept ground freeze active");
}

void testSerializationAndCovarianceJacobians() {
  fp::Snapshot s = snapshot();
  s.state.rot = SO3(Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1., 2., 3.).normalized())));
  s.state.vel = Eigen::Vector3d(2., -0.5, 1.);
  s.state.bg = Eigen::Vector3d(0.01, 0.02, -0.03);
  const fp::Covariance a = fp::Covariance::Random();
  s.covariance = a * a.transpose() * 0.001;
  const auto encoded = fp::encode(s);
  fp::Snapshot decoded;
  std::string error;
  check(fp::decode(encoded, decoded, error), "valid full-state message did not decode");
  check((s.covariance - decoded.covariance).norm() == 0., "covariance changed ordering/major convention");
  check((s.state.rot.coeffs() - decoded.state.rot.coeffs()).norm() < 1e-14, "rotation serialization changed convention");
  check((s.state.grav.get_vect() - decoded.state.grav.get_vect()).norm() < 1e-12, "S2 gravity serialization changed chart");
  fp::Filter original_filter, decoded_filter;
  fp::initializeFilter(original_filter); fp::initializeFilter(decoded_filter);
  original_filter.change_x(s.state); original_filter.change_P(s.covariance);
  decoded_filter.change_x(decoded.state); decoded_filter.change_P(decoded.covariance);
  auto head = sample(s.stamp_ns); auto tail = sample(s.stamp_ns + 5000000LL);
  fp::propagate(original_filter, head, tail, s.noise, s.acceleration_scale);
  fp::propagate(decoded_filter, head, tail, decoded.noise, decoded.acceleration_scale);
  check((original_filter.get_P() - decoded_filter.get_P()).norm() < 1e-12,
    "serialized state/covariance did not preserve IKFoM propagation");
  auto invalid = encoded;
  invalid.covariance[0] = -1.;
  check(!fp::decode(invalid, decoded, error), "non-PSD full-state covariance accepted");
  invalid = encoded; invalid.schema_version = 2;
  check(!fp::decode(invalid, decoded, error), "unknown full-state convention accepted");

  Eigen::Matrix<double, 6, 23> numeric_pose = Eigen::Matrix<double, 6, 23>::Zero();
  Eigen::Matrix<double, 6, 23> numeric_twist = Eigen::Matrix<double, 6, 23>::Zero();
  const Eigen::Vector3d gyro(0.2, -0.1, 0.3);
  const double epsilon = 1e-6;
  for (int column : {0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17}) {
    state_ikfom plus = s.state, minus = s.state;
    if (column < 3) {plus.pos[column] += epsilon; minus.pos[column] -= epsilon;}
    else if (column < 6) {
      const Eigen::Vector3d axis = Eigen::Vector3d::Unit(column - 3);
      plus.rot = SO3(s.state.rot * Eigen::Quaterniond(Eigen::AngleAxisd(epsilon, axis)));
      minus.rot = SO3(s.state.rot * Eigen::Quaterniond(Eigen::AngleAxisd(-epsilon, axis)));
    } else if (column < 15) {plus.vel[column - 12] += epsilon; minus.vel[column - 12] -= epsilon;}
    else {plus.bg[column - 15] += epsilon; minus.bg[column - 15] -= epsilon;}
    const Eigen::Quaterniond world_difference = plus.rot * minus.rot.conjugate();
    numeric_pose.block<3, 1>(0, column) = (plus.pos - minus.pos) / (2. * epsilon);
    numeric_pose.block<3, 1>(3, column) = world_difference.vec() / epsilon;
    numeric_twist.block<3, 1>(0, column) =
      (plus.rot.conjugate() * plus.vel - minus.rot.conjugate() * minus.vel) / (2. * epsilon);
    numeric_twist.block<3, 1>(3, column) = (-plus.bg + minus.bg) / (2. * epsilon);
  }
  const fp::Matrix6 expected_pose = numeric_pose * s.covariance * numeric_pose.transpose();
  const fp::Matrix6 expected_twist = numeric_twist * s.covariance * numeric_twist.transpose();
  check((fp::poseCovariance(s.state, s.covariance) - expected_pose).norm() < 1e-9,
    "pose covariance axes or position/orientation ordering is wrong");
  check((fp::twistCovariance(s.state, s.covariance, Eigen::Matrix3d::Zero()) - expected_twist).norm() < 1e-9,
    "joint child-frame twist covariance Jacobian is wrong");
}

int main() {
  try {
    testGrid();
    testRatesAndScale();
    testDelayedAndFutureCorrection();
    testGapTimeoutEpochAndHold();
    testSerializationAndCovarianceJacobians();
    std::cout << "IMU prediction deterministic checks passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
