#pragma once

#include "use-ikfom.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace fastlio_prediction
{

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
using Covariance = Filter::cov;
using ProcessNoise = Eigen::Matrix<double, 12, 12>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
	Eigen::Matrix3d m;
	m << 0., -v.z(), v.y(), v.z(), 0., -v.x(), -v.y(), v.x(), 0.;
	return m;
}

// A rational grid, rather than rounding a period once, is essential at 30 Hz.
class SourceGrid
{
  public:
	explicit SourceGrid(int rate = 30) : rate_(rate)
	{
		if (rate != 20 && rate != 30 && rate != 40)
		{
			throw std::invalid_argument("prediction.output_rate_hz must be 20, 30 or 40");
		}
	}
	void reset(int64_t origin)
	{
		origin_ = origin;
		index_ = 1;
	}
	int64_t next() const
	{
		// Split seconds/remainder to avoid overflow during long runs.
		return origin_ + static_cast<int64_t>(index_ / rate_) * 1000000000LL +
			   static_cast<int64_t>(((index_ % rate_) * 1000000000ULL + rate_ / 2) / rate_);
	}
	void advance()
	{
		++index_;
	}
	uint64_t skipThrough(int64_t stamp)
	{
		if (next() > stamp)
		{
			return 0;
		}
		const uint64_t before = index_;
		const int64_t delta = std::max<int64_t>(0, stamp - origin_);
		index_ = static_cast<uint64_t>(delta / 1000000000LL) * rate_ +
				 static_cast<uint64_t>(delta % 1000000000LL) * rate_ / 1000000000ULL;
		index_ = std::max(index_, before);
		while (next() <= stamp)
		{
			advance();
		}
		return index_ - before;
	}

  private:
	uint64_t rate_;
	uint64_t index_ = 1;
	int64_t origin_ = 0;
};

struct ImuSample
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int64_t stamp_ns = 0; // Raw IMU timestamp; history never changes timebase.
	Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
	Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
	Eigen::Matrix3d gyro_covariance = Eigen::Matrix3d::Identity() * 1.0e-4;
};

struct Snapshot
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	state_ikfom state;
	Covariance covariance = Covariance::Identity();
	ProcessNoise noise = ProcessNoise::Zero();
	int64_t stamp_ns = 0;
	int64_t correction_stamp_ns = 0;
	int64_t release_stamp_ns = 0;
	int64_t imu_time_offset_ns = 0;
	double acceleration_scale = 1.;
	std::string session_id;
	std::string phase = "WAIT_INIT";
	std::string localization_phase;
	uint64_t epoch = 0;
	uint64_t sequence = 0;
	uint64_t correction_sequence = 0;
	bool initialized = false;
	bool gyro_bias_frozen = false;
	bool recovery_requires_restart = false;
};

struct Output
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int64_t stamp_ns = 0;
	state_ikfom state;
	Covariance covariance;
	Eigen::Vector3d gyro;
	Eigen::Matrix3d gyro_covariance;
};

inline void initializeFilter(Filter &filter)
{
	double limits[23];
	std::fill(limits, limits + 23, 0.001);
	filter.init_dyn_runtime_share(get_f, df_dx, df_dw, 1, limits);
}

inline ImuSample interpolate(const ImuSample &a, const ImuSample &b, int64_t stamp)
{
	if (stamp < a.stamp_ns || stamp > b.stamp_ns || b.stamp_ns <= a.stamp_ns)
	{
		throw std::invalid_argument("IMU interpolation requires a time bracket");
	}
	const double f = double(stamp - a.stamp_ns) / double(b.stamp_ns - a.stamp_ns);
	ImuSample out;
	out.stamp_ns = stamp;
	out.acceleration = (1. - f) * a.acceleration + f * b.acceleration;
	out.gyro = (1. - f) * a.gyro + f * b.gyro;
	// Positive interpolation of the sample uncertainty; no confidence gain is
	// claimed for temporally correlated/interpolated gyro samples.
	out.gyro_covariance = (1. - f) * a.gyro_covariance + f * b.gyro_covariance;
	return out;
}

inline void propagate(Filter &filter, const ImuSample &a, const ImuSample &b, ProcessNoise &noise,
					  double acceleration_scale)
{
	if (b.stamp_ns <= a.stamp_ns)
	{
		return;
	}
	input_ikfom input;
	input.acc = 0.5 * (a.acceleration + b.acceleration) * acceleration_scale;
	input.gyro = 0.5 * (a.gyro + b.gyro);
	double dt = double(b.stamp_ns - a.stamp_ns) * 1e-9;
	filter.predict(dt, noise, input);
}

// Pose errors in ROS fixed world axes, twist jointly in the IMU child axes.
// The downstream rigid-body adapter can shift either 6x6 covariance, including
// its cross blocks, without needing the internal 23-state covariance.
inline Matrix6 poseCovariance(const state_ikfom &state, const Covariance &p)
{
	Eigen::Matrix<double, 6, 23> j = Eigen::Matrix<double, 6, 23>::Zero();
	j.block<3, 3>(0, 0).setIdentity();
	j.block<3, 3>(3, 3) = state.rot.toRotationMatrix();
	return (j * p * j.transpose()).eval();
}

inline Matrix6 twistCovariance(const state_ikfom &state, const Covariance &p,
							   const Eigen::Matrix3d &gyro_sample_covariance)
{
	const Eigen::Matrix3d world_to_imu = state.rot.toRotationMatrix().transpose();
	const Eigen::Vector3d velocity_imu = world_to_imu * state.vel;
	Eigen::Matrix<double, 6, 23> j = Eigen::Matrix<double, 6, 23>::Zero();
	j.block<3, 3>(0, 3) = skew(velocity_imu);
	j.block<3, 3>(0, 12) = world_to_imu;
	j.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
	Matrix6 result = (j * p * j.transpose()).eval();
	result.bottomRightCorner<3, 3>() += gyro_sample_covariance;
	// Instantaneous measurement noise and accumulated state errors are treated
	// as independent here. This covariance is not independent between outputs.
	return 0.5 * (result + result.transpose());
}

struct Settings
{
	int rate_hz = 30;
	int64_t history_ns = 2000000000LL;
	size_t history_samples = 2048;
	int64_t max_imu_gap_ns = 25000000LL;
	int64_t max_correction_age_ns = 500000000LL;
};

// Single-owner core. The ROS node serializes its IMU/state callbacks, but is a
// separate process from the mapping worker. No prediction touches mapper kf.
class Predictor
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	explicit Predictor(const Settings &settings) : settings_(settings), grid_(settings.rate_hz)
	{
		initializeFilter(filter_);
	}

	bool accept(const Snapshot &snapshot,
				const std::function<void(const Output &)> *during_append = nullptr)
	{
		if (have_snapshot_ &&
			((snapshot.session_id == anchor_.session_id && snapshot.epoch < anchor_.epoch) ||
			 retired_sessions_.find(snapshot.session_id) != retired_sessions_.end()))
		{
			++rejected_snapshots_;
			return false;
		}
		const bool new_epoch = !have_snapshot_ || snapshot.session_id != anchor_.session_id ||
							   snapshot.epoch != anchor_.epoch;
		if (!new_epoch && snapshot.sequence <= anchor_.sequence)
		{
			++rejected_snapshots_;
			return false;
		}
		if (!new_epoch && snapshot.stamp_ns < anchor_.stamp_ns)
		{
			++rejected_snapshots_;
			return false;
		}
		if (!new_epoch && snapshot.imu_time_offset_ns != anchor_.imu_time_offset_ns)
		{
			reason_ = "TIMEBASE_CHANGED_WITHOUT_EPOCH";
			active_ = false;
			++rejected_snapshots_;
			return false;
		}
		// DDS delivery order is independent between mapper snapshots and raw
		// IMU. Keep the current predictor alive until its own IMU stream reaches
		// a future correction, instead of discarding intervening grid points.
		if (!new_epoch && active_ && snapshot.initialized && !snapshot.recovery_requires_restart &&
			snapshot.stamp_ns > latestSourceStamp())
		{
			if (!pending_.empty() && (snapshot.sequence <= pending_.back().sequence ||
									  snapshot.stamp_ns < pending_.back().stamp_ns))
			{
				++rejected_snapshots_;
				return false;
			}
			pending_.push_back(snapshot);
			if (pending_.size() > 32)
			{
				pending_.clear();
				active_ = false;
				reason_ = "CORRECTION_QUEUE_OVERFLOW";
			}
			return true;
		}
		if (new_epoch)
		{
			// Existing pre-init IMU is useful for the first initialized snapshot;
			// data from a previous mapper session must never seed a new session.
			if (have_snapshot_)
			{
				history_.clear();
				if (snapshot.session_id != anchor_.session_id)
				{
					retired_sessions_.insert(anchor_.session_id);
				}
			}
			last_published_ns_ = std::numeric_limits<int64_t>::min();
			grid_.reset(snapshot.stamp_ns);
			skipped_outputs_ = 0;
			initialized_grid_ = snapshot.initialized;
			pending_.clear();
		}
		else if (!initialized_grid_ && snapshot.initialized)
		{
			grid_.reset(snapshot.stamp_ns);
			initialized_grid_ = true;
		}
		anchor_ = snapshot;
		have_snapshot_ = true;
		active_ = false;
		if (!snapshot.initialized || snapshot.recovery_requires_restart)
		{
			pending_.clear();
			reason_ =
				snapshot.recovery_requires_restart ? "RECOVERY_REQUIRES_RESTART" : "WAIT_INIT";
			return true;
		}
		state_ikfom state = snapshot.state;
		Covariance covariance = snapshot.covariance;
		filter_.set_frozen_vector_block(15, snapshot.gyro_bias_frozen ? 3 : 0);
		filter_.change_x(state);
		filter_.change_P(covariance);
		current_stamp_ns_ = snapshot.stamp_ns;
		// Do not go backwards in the output stream when delayed corrections are
		// repropagated; the next future grid point receives the new correction.
		skipped_outputs_ += grid_.skipThrough(std::max(snapshot.stamp_ns - 1, last_published_ns_));
		reason_ = "WAIT_IMU_BRACKET";
		active_ = true;
		if (!during_append)
		{
			replaying_ = true;
			processHistory(nullptr);
			replaying_ = false;
		}
		return true;
	}

	void append(const ImuSample &sample, const std::function<void(const Output &)> &publish)
	{
		if (!sample.acceleration.allFinite() || !sample.gyro.allFinite() ||
			!sample.gyro_covariance.allFinite())
		{
			++rejected_imu_;
			reason_ = "NONFINITE_IMU";
			return;
		}
		if (!history_.empty() && sample.stamp_ns <= history_.back().stamp_ns)
		{
			++rejected_imu_;
			return;
		}
		history_.push_back(sample);
		while (history_.size() > 2 &&
			   (history_.size() > settings_.history_samples ||
				history_.back().stamp_ns - history_.front().stamp_ns > settings_.history_ns))
		{
			history_.pop_front();
		}
		while (!pending_.empty() && pending_.front().stamp_ns <= latestSourceStamp())
		{
			const Snapshot next = pending_.front();
			pending_.pop_front();
			// A correction already received over DDS must be applied before any
			// output at or after its source timestamp, even inside an IMU bracket.
			if (active_)
			{
				processHistory(&publish, next.stamp_ns, false);
			}
			accept(next, &publish);
		}
		if (active_)
		{
			processHistory(&publish);
		}
	}

	bool validAt(int64_t stamp) const
	{
		if (!active_ || !have_snapshot_)
		{
			return false;
		}
		if (anchor_.phase == "GROUND_HOLD")
		{
			return true;
		}
		const int64_t reference = anchor_.correction_stamp_ns > 0 ? anchor_.correction_stamp_ns
																  : anchor_.release_stamp_ns;
		return reference > 0 && stamp >= reference &&
			   stamp - reference <= settings_.max_correction_age_ns;
	}
	const Snapshot &anchor() const
	{
		return anchor_;
	}
	const std::string &reason() const
	{
		return reason_;
	}
	bool active() const
	{
		return active_;
	}
	int64_t latestSourceStamp() const
	{
		return history_.empty() ? 0 : history_.back().stamp_ns + anchor_.imu_time_offset_ns;
	}
	int64_t lastPublishedStamp() const
	{
		return last_published_ns_;
	}
	uint64_t skippedOutputs() const
	{
		return skipped_outputs_;
	}
	uint64_t rejectedImu() const
	{
		return rejected_imu_;
	}
	uint64_t rejectedSnapshots() const
	{
		return rejected_snapshots_;
	}
	int64_t largestGap() const
	{
		return largest_gap_ns_;
	}

  private:
	void emitPrediction(Filter &filter, const ImuSample &sample,
						const std::function<void(const Output &)> &publish, bool hold)
	{
		Output out;
		out.stamp_ns = sample.stamp_ns;
		out.state = filter.get_x();
		out.covariance = filter.get_P();
		out.gyro = sample.gyro;
		out.gyro_covariance = sample.gyro_covariance;
		publish(out);
		last_published_ns_ = sample.stamp_ns;
		reason_ = hold ? "GROUND_HOLD" : "OK";
	}

	ImuSample corrected(const ImuSample &raw) const
	{
		ImuSample sample = raw;
		sample.stamp_ns += anchor_.imu_time_offset_ns;
		return sample;
	}

	void processHistory(const std::function<void(const Output &)> *publish,
						int64_t end_ns = std::numeric_limits<int64_t>::max(),
						bool include_end = true)
	{
		if (history_.size() < 2)
		{
			return;
		}
		const int64_t earliest = history_.front().stamp_ns + anchor_.imu_time_offset_ns;
		if (current_stamp_ns_ < earliest)
		{
			active_ = false;
			reason_ = "HISTORY_UNDERRUN";
			return;
		}
		for (size_t i = 1; i < history_.size(); ++i)
		{
			const ImuSample raw_head = corrected(history_[i - 1]);
			const ImuSample original_tail = corrected(history_[i]);
			if (original_tail.stamp_ns <= current_stamp_ns_)
			{
				// Applying a queued correction can leave its exact grid point at
				// the last available IMU. Emit the corrected state now; waiting for
				// another IMU adds latency and loses the final point at shutdown.
				if (publish && original_tail.stamp_ns == current_stamp_ns_ &&
					grid_.next() == current_stamp_ns_ && current_stamp_ns_ > last_published_ns_ &&
					(current_stamp_ns_ < end_ns || (include_end && current_stamp_ns_ == end_ns)) &&
					validAt(current_stamp_ns_))
				{
					emitPrediction(filter_, original_tail, *publish,
								   anchor_.phase == "GROUND_HOLD");
					grid_.advance();
				}
				continue;
			}
			if (current_stamp_ns_ >= end_ns)
			{
				return;
			}
			if (raw_head.stamp_ns > current_stamp_ns_)
			{
				active_ = false;
				reason_ = "HISTORY_UNDERRUN";
				return;
			}
			const int64_t gap = original_tail.stamp_ns - raw_head.stamp_ns;
			largest_gap_ns_ = std::max(largest_gap_ns_, gap);
			if (gap > settings_.max_imu_gap_ns)
			{
				active_ = false;
				reason_ = "IMU_GAP";
				return;
			}
			const ImuSample tail = original_tail.stamp_ns > end_ns
									   ? interpolate(raw_head, original_tail, end_ns)
									   : original_tail;
			const ImuSample head = current_stamp_ns_ == raw_head.stamp_ns
									   ? raw_head
									   : interpolate(raw_head, original_tail, current_stamp_ns_);
			const bool hold = anchor_.phase == "GROUND_HOLD";
			while (grid_.next() <= tail.stamp_ns)
			{
				const int64_t target = grid_.next();
				if (!include_end && target == end_ns)
				{
					break;
				}
				if (target >= current_stamp_ns_ && target > last_published_ns_ && publish &&
					validAt(target))
				{
					Filter partial = filter_;
					const ImuSample at_target = interpolate(head, tail, target);
					if (!hold)
					{
						propagate(partial, head, at_target, anchor_.noise,
								  anchor_.acceleration_scale);
					}
					emitPrediction(partial, at_target, *publish, hold);
				}
				else if (!replaying_)
				{
					++skipped_outputs_;
					if (!validAt(target))
					{
						reason_ = "CORRECTION_STALE";
					}
				}
				grid_.advance();
			}
			if (!hold)
			{
				propagate(filter_, head, tail, anchor_.noise, anchor_.acceleration_scale);
			}
			current_stamp_ns_ = tail.stamp_ns;
		}
	}

	Settings settings_;
	SourceGrid grid_;
	Filter filter_;
	Snapshot anchor_;
	std::deque<ImuSample, Eigen::aligned_allocator<ImuSample>> history_;
	std::deque<Snapshot, Eigen::aligned_allocator<Snapshot>> pending_;
	std::unordered_set<std::string> retired_sessions_;
	int64_t current_stamp_ns_ = 0;
	int64_t last_published_ns_ = std::numeric_limits<int64_t>::min();
	int64_t largest_gap_ns_ = 0;
	uint64_t skipped_outputs_ = 0;
	uint64_t rejected_imu_ = 0;
	uint64_t rejected_snapshots_ = 0;
	bool have_snapshot_ = false;
	bool initialized_grid_ = false;
	bool active_ = false;
	bool replaying_ = false;
	std::string reason_ = "WAIT_INIT";
};

} // namespace fastlio_prediction
