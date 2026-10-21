#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

// Timestamp-only observer: callback gaps are measured by C++ steady_clock,
// independently of rosbag's writer and the Python analysis process.
class StreamTimingNode : public rclcpp::Node
{
  public:
	StreamTimingNode() : Node("fastlio_stream_probe")
	{
		const auto path = declare_parameter<std::string>("output", "");
		ready_path_ = declare_parameter<std::string>("ready_file", "");
		required_ = declare_parameter<std::vector<std::string>>(
			"required_topics", {"/livox/imu", "/livox/lidar", "/Odometry"});
		if (path.empty() || ready_path_.empty())
		{
			throw std::invalid_argument("output and ready_file are required");
		}
		stream_.open(path, std::ios::out | std::ios::trunc);
		stream_.exceptions(std::ios::badbit | std::ios::failbit);
		observe<sensor_msgs::msg::Imu>("/livox/imu");
		observe<livox_interfaces::msg::CustomMsg>("/livox/lidar");
		for (const auto &topic :
			 {"/Odometry", "/lio/mapping_odometry", "/lio/imu_prediction/raw", "/lio/body_enu"})
		{
			observe<nav_msgs::msg::Odometry>(topic);
		}
		observe<geometry_msgs::msg::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov");
		timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { updateReadiness(); });
	}

  private:
	template <typename Message> void observe(const std::string &topic)
	{
		subscriptions_.push_back(create_subscription<Message>(
			topic, rclcpp::QoS(2048).reliable(),
			[this, topic](typename Message::ConstSharedPtr message)
			{
				const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
										   std::chrono::steady_clock::now().time_since_epoch())
										   .count();
				const auto &stamp = message->header.stamp;
				const int64_t source_ns = int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
				stream_ << "{\"topic\":\"" << topic << "\",\"stamp_ns\":" << source_ns
						<< ",\"steady_ns\":" << steady_ns
						<< ",\"ros_ns\":" << get_clock()->now().nanoseconds() << "}\n";
				++counts_[topic];
			}));
	}

	void updateReadiness()
	{
		stream_.flush();
		if (ready_)
		{
			return;
		}
		for (const auto &topic : required_)
		{
			if (counts_[topic] == 0)
			{
				return;
			}
		}
		std::ofstream ready(ready_path_);
		ready.exceptions(std::ios::badbit | std::ios::failbit);
		ready << "ready\n";
		ready_ = true;
	}

	std::ofstream stream_;
	std::string ready_path_;
	std::vector<std::string> required_;
	std::map<std::string, size_t> counts_;
	std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool ready_ = false;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		rclcpp::spin(std::make_shared<StreamTimingNode>());
	}
	catch (const std::exception &error)
	{
		RCLCPP_ERROR(rclcpp::get_logger("fastlio_stream_probe"), "%s", error.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
