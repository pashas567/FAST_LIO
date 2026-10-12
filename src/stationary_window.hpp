#pragma once

#include <deque>
#include <cmath>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Core>

// Time-based initialization, independent of whether the lidar sees any surface.
// Evaluate 200 ms means: contact vibration must not be mistaken for sustained
// acceleration, but its impulse must already be preserved by the sensor path.
class StationaryWindow
{
public:
	using Sample = sensor_msgs::msg::Imu::ConstSharedPtr;
	std::deque<Sample> samples;
	const char* reason = "collecting";
	double acceleration_scale = 1.0;

	static int64_t Stamp(const Sample& sample)
	{
		return int64_t(sample->header.stamp.sec) * 1000000000 + sample->header.stamp.nanosec;
	}

	void Clear()
	{
		samples.clear();
		reason = "collecting";
	}

	bool Append(const Sample& sample)
	{
		const auto& a = sample->linear_acceleration;
		const auto& w = sample->angular_velocity;
		if (!Eigen::Vector3d(a.x, a.y, a.z).allFinite() || !Eigen::Vector3d(w.x, w.y, w.z).allFinite())
		{
			Clear();
			reason = "nonfinite";
			return false;
		}
		if (!samples.empty())
		{
			const int64_t interval = Stamp(sample) - Stamp(samples.back());
			if (interval <= 0 || interval > 7500000)
			{
				Clear();
				reason = "time_gap";
			}
		}
		samples.push_back(sample);
		while (samples.size() > 1 && Stamp(sample) - Stamp(samples[1]) >= 2000000000)
		{
			samples.pop_front();
		}
		if (samples.size() > 2001)
		{
			Clear();
			reason = "unexpected_rate";
			return false;
		}
		if (Stamp(sample) - Stamp(samples.front()) < 2000000000)
		{
			return false;
		}
		constexpr int BlockCount = 10;
		constexpr int64_t BlockNs = 200000000;
		Eigen::Vector3d acceleration[BlockCount];
		Eigen::Vector3d gyro[BlockCount];
		int counts[BlockCount] = {};
		for (int index = 0; index < BlockCount; ++index)
		{
			acceleration[index].setZero();
			gyro[index].setZero();
		}
		const int64_t begin = Stamp(sample) - 2000000000;
		for (const auto& value : samples)
		{
			const int64_t offset = Stamp(value) - begin;
			if (offset < 0 || offset >= 2000000000)
			{
				continue;
			}
			const int index = int(offset / BlockNs);
			const auto& force = value->linear_acceleration;
			const auto& omega = value->angular_velocity;
			acceleration[index] += acceleration_scale * Eigen::Vector3d(force.x, force.y, force.z);
			gyro[index] += Eigen::Vector3d(omega.x, omega.y, omega.z);
			++counts[index];
		}
		Eigen::Vector3d mean = Eigen::Vector3d::Zero();
		for (int index = 0; index < BlockCount; ++index)
		{
			if (counts[index] < 20)
			{
				reason = "sparse_window";
				return false;
			}
			acceleration[index] /= counts[index];
			gyro[index] /= counts[index];
			mean += acceleration[index] / BlockCount;
		}
		for (int index = 0; index < BlockCount; ++index)
		{
			if (std::abs(mean.norm() - 9.81) > 0.3 || (acceleration[index] - mean).norm() > 0.3
				|| gyro[index].norm() > 0.02)
			{
				reason = "motion";
				return false;
			}
		}
		reason = "stationary";
		return true;
	}
};
