#include <chrono>
#include <memory>
#include <string>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_prediction_state.hpp"

namespace fp = fastlio_prediction;

class ImuPredictionNode : public rclcpp::Node {
public:
  ImuPredictionNode() : Node("fastlio_imu_prediction") {
    enabled_ = declare_parameter<bool>("prediction.enabled", false);
    fp::Settings settings;
    const double output_rate = declare_parameter<double>("prediction.output_rate_hz", 30.0);
    if (output_rate != 20.0 && output_rate != 30.0 && output_rate != 40.0) {
      throw std::invalid_argument("prediction.output_rate_hz must be 20.0, 30.0 or 40.0");
    }
    settings.rate_hz = static_cast<int>(output_rate);
    settings.history_ns = secondsParameter("prediction.history_seconds", 2.0);
    const int max_samples = declare_parameter<int>("prediction.max_history_samples", 2048);
    if (max_samples < 4 || max_samples > 100000) {
      throw std::invalid_argument("prediction.max_history_samples must be in [4, 100000]");
    }
    settings.history_samples = static_cast<size_t>(max_samples);
    settings.max_imu_gap_ns = secondsParameter("prediction.max_imu_gap_s", 0.025);
    settings.max_correction_age_ns = secondsParameter("prediction.max_correction_age_s", 0.5);
    imu_timeout_ns_ = secondsParameter("prediction.imu_timeout_s", 0.05);
    correction_warn_ns_ = secondsParameter("prediction.correction_warn_s", 0.3);
    if (correction_warn_ns_ > settings.max_correction_age_ns ||
        settings.history_ns < settings.max_correction_age_ns ||
        settings.max_imu_gap_ns > imu_timeout_ns_) {
      throw std::invalid_argument("inconsistent prediction timeouts/history");
    }
    const double gyro_stddev = declare_parameter<double>("prediction.gyro_sample_stddev", 0.01);
    if (!std::isfinite(gyro_stddev) || gyro_stddev <= 0.) {
      throw std::invalid_argument("prediction.gyro_sample_stddev must be positive");
    }
    fallback_gyro_variance_ = gyro_stddev * gyro_stddev;
    predictor_.reset(new fp::Predictor(settings));
    const std::string input = declare_parameter<std::string>("imu_topic", "/livox/imu");
    const std::string state = declare_parameter<std::string>("state_topic", "/lio/filter_state");
    const std::string output = declare_parameter<std::string>("output_topic", "/lio/imu_prediction/raw");
    const std::string status = declare_parameter<std::string>("status_topic", "/lio/prediction_status");
    output_ = create_publisher<nav_msgs::msg::Odometry>(output, rclcpp::QoS(10).reliable());
    status_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status, rclcpp::QoS(20).reliable());
    if (enabled_) {
      // The default mutually-exclusive group serializes the two lightweight
      // callbacks. This process never shares the mapper executor/ICP lock.
      state_sub_ = create_subscription<fast_lio::msg::FilterState>(state,
        rclcpp::QoS(20).reliable().transient_local(),
        [this](fast_lio::msg::FilterState::ConstSharedPtr msg) {onState(*msg);});
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(input,
        rclcpp::SensorDataQoS().keep_last(400),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(*msg);});
    }
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publishStatus();});
    RCLCPP_INFO(get_logger(), "IMU prediction %s; source-time grid %d Hz, no future extrapolation",
      enabled_ ? "enabled" : "disabled", settings.rate_hz);
    publishStatus();
  }

private:
  int64_t secondsParameter(const std::string & name, double initial) {
    const double value = declare_parameter<double>(name, initial);
    if (!std::isfinite(value) || value <= 0. || value > 3600.) {
      throw std::invalid_argument(name + " must be finite, positive and <= 3600");
    }
    return static_cast<int64_t>(std::llround(value * 1e9));
  }

  void onState(const fast_lio::msg::FilterState & message) {
    fp::Snapshot state;
    std::string error;
    if (!fp::decode(message, state, error)) {
      bad_snapshot_ = true;
      state_error_ = error;
      publishStatus();
      return;
    }
    const auto & previous = predictor_->anchor();
    const bool new_epoch = previous.session_id != state.session_id || previous.epoch != state.epoch;
    if (predictor_->accept(state)) {
      bad_snapshot_ = false;
      numeric_failure_ = false;
      state_error_.clear();
      if (new_epoch) {latest_output_ns_ = 0;}
    }
    publishStatus();
  }

  void onImu(const sensor_msgs::msg::Imu & message) {
    fp::ImuSample sample;
    sample.stamp_ns = fp::stampNs(message.header.stamp);
    sample.acceleration = fp::eigenVector(message.linear_acceleration);
    sample.gyro = fp::eigenVector(message.angular_velocity);
    sample.gyro_covariance = Eigen::Matrix3d::Identity() * fallback_gyro_variance_;
    Eigen::Matrix3d gyro_covariance;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        gyro_covariance(row, col) = message.angular_velocity_covariance[row * 3 + col];
      }
    }
    if (gyro_covariance.allFinite() && gyro_covariance.trace() > 0. &&
        gyro_covariance.isApprox(gyro_covariance.transpose(), 1e-8)) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(gyro_covariance, Eigen::EigenvaluesOnly);
      if (eigen.info() == Eigen::Success && eigen.eigenvalues().minCoeff() >= 0.) {
        sample.gyro_covariance = gyro_covariance;
      }
    }
    // Receipt time only detects loss of transport. It never becomes a pose
    // timestamp or conceals SDK1's source-time bursts.
    last_imu_receipt_ = std::chrono::steady_clock::now();
    have_imu_ = true;
    predictor_->append(sample, [this](const fp::Output & out) {publishOutput(out);});
    if (!predictor_->active()) {publishStatus();}
  }

  void publishOutput(const fp::Output & out) {
    if (bad_snapshot_ || numeric_failure_) {return;}
    const auto & state = out.state;
    if (!state.pos.allFinite() || !state.vel.allFinite() || !state.rot.coeffs().allFinite() ||
        !out.covariance.allFinite()) {
      numeric_failure_ = true;
      state_error_ = "NONFINITE_PREDICTION";
      publishStatus();
      return;
    }
    nav_msgs::msg::Odometry message;
    message.header.frame_id = "camera_init";
    message.child_frame_id = "fastlio_imu";
    message.header.stamp = fp::rosStamp(out.stamp_ns);
    message.pose.pose.position = fp::vectorMessage<geometry_msgs::msg::Point>(state.pos);
    message.pose.pose.orientation = fp::quaternionMessage(state.rot);
    const bool hold = predictor_->anchor().phase == "GROUND_HOLD";
    const Eigen::Vector3d velocity = state.rot.conjugate() * state.vel;
    const Eigen::Vector3d angular = hold ? Eigen::Vector3d::Zero().eval() :
      Eigen::Vector3d(out.gyro - state.bg);
    message.twist.twist.linear = fp::vectorMessage<geometry_msgs::msg::Vector3>(velocity);
    message.twist.twist.angular = fp::vectorMessage<geometry_msgs::msg::Vector3>(angular);
    const fp::Matrix6 pose = fp::poseCovariance(state, out.covariance);
    const fp::Matrix6 twist = fp::twistCovariance(state, out.covariance, out.gyro_covariance);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        message.pose.covariance[row * 6 + col] = pose(row, col);
        message.twist.covariance[row * 6 + col] = twist(row, col);
      }
    }
    latest_output_ns_ = out.stamp_ns;
    output_->publish(message);
    publishStatus();
  }

  void publishStatus() {
    const auto & anchor = predictor_->anchor();
    const int64_t source = predictor_->latestSourceStamp();
    const int64_t now_ns = get_clock()->now().nanoseconds();
    const int64_t receipt_age = have_imu_ ?
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - last_imu_receipt_).count() :
      std::numeric_limits<int64_t>::max();
    const int64_t source_age = source > 0 ? now_ns - source : std::numeric_limits<int64_t>::max();
    const bool imu_fresh = have_imu_ && receipt_age <= imu_timeout_ns_ &&
      source_age >= -1000000LL && source_age <= imu_timeout_ns_;
    const int64_t correction_age = anchor.correction_stamp_ns > 0 && source >= anchor.correction_stamp_ns ?
      source - anchor.correction_stamp_ns : -1;
    const bool valid = enabled_ && latest_output_ns_ > 0 && imu_fresh &&
      predictor_->validAt(source) && !bad_snapshot_ && !numeric_failure_ &&
      !anchor.recovery_requires_restart;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = fp::rosStamp(now_ns);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fastlio_prediction";
    status.hardware_id = "fast_lio";
    status.level = valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = !enabled_ ? "DISABLED" :
      (!state_error_.empty() ? state_error_ : (!imu_fresh && have_imu_ ? "IMU_STALE" : predictor_->reason()));
    if (valid && correction_age > correction_warn_ns_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "CORRECTION_AGING";
    }
    const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry; entry.key = key; entry.value = value;
      status.values.push_back(entry);
    };
    add("valid", valid ? "true" : "false");
    add("phase", anchor.phase);
    add("session_id", anchor.session_id);
    add("epoch", std::to_string(anchor.epoch));
    add("source_stamp_ns", std::to_string(source));
    add("output_stamp_ns", std::to_string(latest_output_ns_));
    add("last_correction_stamp_ns", std::to_string(anchor.correction_stamp_ns));
    add("correction_sequence", std::to_string(anchor.correction_sequence));
    add("correction_age_s", correction_age < 0 ? "-1" : std::to_string(correction_age * 1e-9));
    add("skipped_outputs", std::to_string(predictor_->skippedOutputs()));
    add("rejected_imu", std::to_string(predictor_->rejectedImu()));
    add("rejected_snapshots", std::to_string(predictor_->rejectedSnapshots()));
    add("largest_imu_gap_s", std::to_string(predictor_->largestGap() * 1e-9));
    add("imu_source_age_s", source_age == std::numeric_limits<int64_t>::max() ? "-1" : std::to_string(source_age * 1e-9));
    add("recovery_requires_restart", anchor.recovery_requires_restart ? "true" : "false");
    add("gyro_fallback_variance", std::to_string(fallback_gyro_variance_));
    array.status.push_back(std::move(status));
    status_->publish(array);
  }

  bool enabled_ = false;
  bool have_imu_ = false;
  bool bad_snapshot_ = false;
  bool numeric_failure_ = false;
  int64_t latest_output_ns_ = 0;
  int64_t imu_timeout_ns_ = 50000000LL;
  int64_t correction_warn_ns_ = 300000000LL;
  double fallback_gyro_variance_ = 1e-4;
  std::string state_error_;
  std::chrono::steady_clock::time_point last_imu_receipt_;
  std::unique_ptr<fp::Predictor> predictor_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<fast_lio::msg::FilterState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuPredictionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fastlio_imu_prediction"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
