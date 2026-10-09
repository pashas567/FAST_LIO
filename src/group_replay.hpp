#pragma once

#include <cstring>
#include <fstream>
#include <stdexcept>
#include <rclcpp/serialization.hpp>

// Offline input adapter only. The live and replay paths share all preprocessing,
// IMU propagation, deskew, map updates and iterated filter corrections.
class GroupReplay
{
public:
	explicit GroupReplay(const std::string& path) : input_(path, std::ios::binary)
	{
		char magic[8];
		input_.read(magic, sizeof(magic));
		const bool ground_start = std::memcmp(magic, "FLGR0002", sizeof(magic)) == 0;
		if (!input_ || (!ground_start && std::memcmp(magic, "FLGR0001", sizeof(magic)) != 0))
		{
			throw std::runtime_error("Invalid fixed-group replay header");
		}
		if (ground_start)
		{
			const uint32_t count = Read<uint32_t>();
			if (count < 2 || count > 10000)
			{
				throw std::runtime_error("Invalid ground initialization sample count");
			}
			for (uint32_t index = 0; index < count; ++index)
			{
				GroundImu.push_back(Message<sensor_msgs::msg::Imu>());
			}
			LastGroundImu = Message<sensor_msgs::msg::Imu>();
		}
	}

	std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> GroundImu;
	sensor_msgs::msg::Imu::ConstSharedPtr LastGroundImu;

	bool Next(MeasureGroup& group, Preprocess& preprocess, bool check_point_count)
	{
		if (input_.peek() == std::char_traits<char>::eof())
		{
			return false;
		}
		const uint64_t source_group = Read<uint64_t>();
		const uint64_t source_cloud = Read<uint64_t>();
		group.lidar_beg_time = Read<double>();
		group.lidar_end_time = Read<double>();
		const uint32_t points = Read<uint32_t>();
		const uint32_t count = Read<uint32_t>();
		if (count > 10000 || !std::isfinite(group.lidar_beg_time)
			|| !std::isfinite(group.lidar_end_time) || group.lidar_end_time < group.lidar_beg_time)
		{
			throw std::runtime_error("Invalid replay group boundaries");
		}
		auto cloud = Message<livox_ros_driver2::msg::CustomMsg>();
		group.lidar.reset(new PointCloudXYZI());
		const uint64_t cloud_trace = StartupTrace::Get().Input(2, *cloud, group.lidar.get());
		preprocess.process(cloud, group.lidar);
		if (check_point_count && group.lidar->size() != points)
		{
			throw std::runtime_error("Replay preprocessing differs from recorded group");
		}
		group.imu.clear();
		for (uint32_t index = 0; index < count; ++index)
		{
			const uint64_t source_imu = Read<uint64_t>();
			auto imu = Message<sensor_msgs::msg::Imu>();
			const uint64_t trace_imu = StartupTrace::Get().Input(1, *imu, imu.get());
			const int64_t stamp = int64_t(imu->header.stamp.sec) * 1000000000 + imu->header.stamp.nanosec;
			StartupTrace::Get().Event("imu_callback," + std::to_string(trace_imu)
				+ "," + std::to_string(stamp) + "," + std::to_string(stamp));
			StartupTrace::Get().Event("replay_imu," + std::to_string(source_imu)
				+ "," + std::to_string(trace_imu));
			group.imu.push_back(std::move(imu));
		}
		StartupTrace::Get().Event("replay_group," + std::to_string(source_group)
			+ "," + std::to_string(source_cloud) + "," + std::to_string(cloud_trace));
		return true;
	}

private:
	template<class Value>
	Value Read()
	{
		Value value;
		input_.read(reinterpret_cast<char*>(&value), sizeof(value));
		if (!input_)
		{
			throw std::runtime_error("Truncated replay input");
		}
		return value;
	}

	template<class Type>
	std::unique_ptr<Type> Message()
	{
		const uint32_t length = Read<uint32_t>();
		if (length == 0 || length > 32 * 1024 * 1024)
		{
			throw std::runtime_error("Invalid replay message length");
		}
		rclcpp::SerializedMessage serialized(length);
		auto& data = serialized.get_rcl_serialized_message();
		input_.read(reinterpret_cast<char*>(data.buffer), length);
		if (!input_)
		{
			throw std::runtime_error("Truncated replay message");
		}
		data.buffer_length = length;
		std::unique_ptr<Type> message(new Type());
		rclcpp::Serialization<Type> serializer;
		serializer.deserialize_message(&serialized, message.get());
		return message;
	}

	std::ifstream input_;
};
