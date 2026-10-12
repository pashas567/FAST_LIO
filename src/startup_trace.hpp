#pragma once

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <rclcpp/serialization.hpp>

// Observation only. All hot-path records stay in bounded memory until shutdown.
// The CDR file preserves callback order; events preserve selected IMU groups.
class StartupTrace
{
public:
	static StartupTrace& Get()
	{
		static StartupTrace trace;
		return trace;
	}

	bool Active() const
	{
		return !directory_.empty() && !saved_
			&& std::chrono::duration<double>(Clock::now() - started_).count() < duration_s_;
	}

	template<class Message>
	uint64_t Input(uint32_t kind, const Message& message, const void* identity = nullptr)
	{
		if (!Active())
		{
			return 0;
		}
		const uint64_t receive_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		if (!input_reference_.empty())
		{
			const uint64_t sequence = ++sequence_;
			if (identity)
			{
				identities_[identity] = sequence;
			}
			Event("referenced_input," + std::to_string(sequence) + ","
				+ std::to_string(receive_ns) + "," + std::to_string(kind));
			return sequence;
		}
		rclcpp::SerializedMessage serialized;
		rclcpp::Serialization<Message> serializer;
		serializer.serialize_message(&message, &serialized);
		const auto& data = serialized.get_rcl_serialized_message();
		const uint64_t sequence = ++sequence_;
		if (bytes_ + data.buffer_length > max_bytes_)
		{
			++dropped_;
			if (identity)
			{
				identities_.erase(identity);
			}
			return 0;
		}
		Record record;
		record.sequence = sequence;
		record.receive_ns = receive_ns;
		record.kind = kind;
		record.data.assign(reinterpret_cast<const char*>(data.buffer), data.buffer_length);
		bytes_ += record.data.size();
		records_.push_back(std::move(record));
		if (identity)
		{
			identities_[identity] = sequence;
		}
		return sequence;
	}

	void Event(const std::string& text)
	{
		if (!Active())
		{
			return;
		}
		if (event_bytes_ + text.size() + 1 > max_event_bytes_)
		{
			++dropped_;
			return;
		}
		events_.push_back(text);
		event_bytes_ += text.size() + 1;
	}

	template<class Group>
	void GroupSelected(const Group& group)
	{
		if (!Active())
		{
			return;
		}
		++group_;
		std::ostringstream line;
		line << std::setprecision(17) << "group," << group_ << ','
			<< group.lidar_beg_time << ',' << group.lidar_end_time << ','
			<< group.lidar->size() << ',' << group.imu.size();
		Event(line.str());
		const auto cloud_identity = identities_.find(group.lidar.get());
		uint64_t hash = 14695981039346656037ull;
		for (const auto& point : group.lidar->points)
		{
			const float values[] = {point.x, point.y, point.z, point.intensity, point.curvature};
			for (const float value : values)
			{
				const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
				for (size_t byte = 0; byte < sizeof(value); ++byte)
				{
					hash ^= bytes[byte];
					hash *= 1099511628211ull;
				}
			}
		}
		Event("cloud," + std::to_string(group_) + ","
			+ std::to_string(cloud_identity == identities_.end() ? 0 : cloud_identity->second)
			+ "," + std::to_string(hash));
		for (size_t index = 0; index < group.imu.size(); ++index)
		{
			const auto& imu = group.imu[index];
			const auto found = identities_.find(imu.get());
			std::ostringstream member;
			member << "member," << group_ << ',' << index << ','
				<< (found == identities_.end() ? 0 : found->second) << ','
				<< int64_t(imu->header.stamp.sec) * 1000000000 + imu->header.stamp.nanosec;
			Event(member.str());
		}
	}

	uint64_t GroupId() const
	{
		return group_;
	}

	void Identify(const void* identity, uint64_t sequence)
	{
		if (Active() && identity && sequence != 0)
		{
			identities_[identity] = sequence;
		}
	}

	void Save()
	{
		if (directory_.empty() || saved_)
		{
			return;
		}
		saved_ = true;
		// ROS launch may send another SIGINT while shutdown flushes this buffer.
		const auto previous_interrupt = std::signal(SIGINT, SIG_IGN);
		std::ofstream inputs(directory_ + "/inputs.cdr", std::ios::binary);
		std::ofstream events(directory_ + "/events.csv");
		inputs.write("FLTR0001", 8);
		for (const auto& record : records_)
		{
			Write(inputs, record.sequence);
			Write(inputs, record.receive_ns);
			Write(inputs, record.kind);
			Write(inputs, uint32_t(record.data.size()));
			inputs.write(record.data.data(), record.data.size());
		}
		for (const auto& line : events_)
		{
			events << line << '\n';
		}
		inputs.flush();
		events.flush();
		const bool successful = inputs.good() && events.good();
		std::ofstream manifest(directory_ + "/manifest.json");
		manifest << "{\"schema\":1,\"duration_s\":" << duration_s_
			<< ",\"records\":" << records_.size() << ",\"groups\":" << group_
			<< ",\"input_bytes\":" << bytes_ << ",\"event_bytes\":" << event_bytes_
			<< ",\"max_input_bytes\":" << max_bytes_ << ",\"max_event_bytes\":" << max_event_bytes_
			<< ",\"input_reference\":" << std::quoted(input_reference_)
			<< ",\"referenced_records\":" << (input_reference_.empty() ? 0 : sequence_)
			<< ",\"dropped_trace_records\":" << dropped_
			<< ",\"saved\":" << (successful ? "true" : "false") << "}\n";
		if (!successful || !manifest.good())
		{
			std::cerr << "Startup trace save failed: " << directory_ << std::endl;
		}
		std::signal(SIGINT, previous_interrupt);
	}

	~StartupTrace()
	{
		Save();
	}

private:
	using Clock = std::chrono::steady_clock;
	struct Record
	{
		uint64_t sequence = 0;
		uint64_t receive_ns = 0;
		uint32_t kind = 0;
		std::string data;
	};

	StartupTrace() : started_(Clock::now())
	{
		const char* directory = std::getenv("FASTLIO_STARTUP_TRACE_DIR");
		if (directory)
		{
			directory_ = directory;
		}
		const char* duration = std::getenv("FASTLIO_STARTUP_TRACE_SECONDS");
		if (duration)
		{
			duration_s_ = std::stod(duration);
		}
		max_bytes_ = Budget("FASTLIO_STARTUP_TRACE_MAX_MB", 256, 2048);
		max_event_bytes_ = Budget("FASTLIO_STARTUP_TRACE_EVENT_MB", 16, 256);
		const char* reference = std::getenv("FASTLIO_STARTUP_TRACE_INPUT_REFERENCE");
		if (reference && reference[0] != '\0')
		{
			std::ifstream input(reference, std::ios::binary);
			char magic[8];
			if (!input.read(magic, sizeof(magic)) || std::string(magic, sizeof(magic)) != "FLGR0002")
			{
				throw std::invalid_argument("Referenced trace inputs require frozen ground replay groups");
			}
			input_reference_ = reference;
		}
	}

	static size_t Budget(const char* name, unsigned long default_mb, unsigned long maximum_mb)
	{
		const char* setting = std::getenv(name);
		const unsigned long megabytes = setting ? std::stoul(setting) : default_mb;
		if (megabytes == 0 || megabytes > maximum_mb)
		{
			throw std::invalid_argument(std::string(name) + " is outside the trace memory limit");
		}
		return megabytes * 1024 * 1024;
	}

	template<class Value>
	static void Write(std::ofstream& file, const Value& value)
	{
		file.write(reinterpret_cast<const char*>(&value), sizeof(value));
	}

	std::string directory_;
	std::string input_reference_;
	Clock::time_point started_;
	double duration_s_ = 35.0;
	bool saved_ = false;
	size_t bytes_ = 0;
	size_t event_bytes_ = 0;
	size_t max_bytes_ = 256 * 1024 * 1024;
	size_t max_event_bytes_ = 16 * 1024 * 1024;
	uint64_t sequence_ = 0;
	uint64_t group_ = 0;
	uint64_t dropped_ = 0;
	std::vector<Record> records_;
	std::vector<std::string> events_;
	std::unordered_map<const void*, uint64_t> identities_;
};
