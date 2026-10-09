#include <omp.h>
#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
#include "stationary_window.hpp"
#include <Eigen/Eigenvalues>
#include <stdexcept>

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

void Check(bool condition, const char* description)
{
	if (!condition)
	{
		throw std::runtime_error(description);
	}
}

void PositionMeasurement(state_ikfom& state, esekfom::dyn_share_datastruct<double>& measurement)
{
	measurement.valid = true;
	measurement.h_x = Eigen::MatrixXd::Zero(1, 12);
	measurement.h_x(0, 0) = 1.0;
	measurement.h = Eigen::VectorXd::Constant(1, 1.0 - state.pos[0]);
}

void Initialize(Filter& filter)
{
	double limits[23];
	std::fill(limits, limits + 23, 1e-10);
	filter.init_dyn_share(get_f, df_dx, df_dw, PositionMeasurement, 4, limits);
}

void TestSchmidtCovariance()
{
	Filter filter;
	Initialize(filter);
	Filter::cov prior = Filter::cov::Identity();
	prior(15, 15) = 0.04;
	prior(0, 15) = 0.1;
	prior(15, 0) = 0.1;
	filter.change_P(prior);
	filter.set_frozen_vector_block(15, 3);
	double elapsed = 0.0;
	filter.update_iterated_dyn_share_modified(0.01, elapsed);
	const auto& posterior = filter.get_P();
	Check(std::abs(filter.get_x().pos[0] - 1.0 / 1.01) < 1e-9, "Active state did not update");
	Check(filter.get_x().bg.norm() == 0.0, "Frozen bias mean changed");
	Check(std::abs(posterior(15, 15) - 0.04) < 1e-12, "Nuisance uncertainty was lost");
	Check(std::abs(posterior(0, 15) - 0.1 / 101.0) < 1e-9, "Schmidt cross covariance is wrong");
	Check((posterior - posterior.transpose()).norm() < 1e-10, "Posterior is asymmetric");
	Eigen::SelfAdjointEigenSolver<Filter::cov> eigenvalues(posterior);
	Check(eigenvalues.eigenvalues().minCoeff() >= -1e-12, "Posterior is not positive semidefinite");
}

MeasureGroup StationaryGroup(double begin, double end, double imu_begin, int count)
{
	MeasureGroup group;
	group.lidar_beg_time = begin;
	group.lidar_end_time = end;
	group.lidar.reset(new PointCloudXYZI());
	PointType point{};
	point.x = 5.0f;
	point.curvature = float((end - begin) * 1000.0);
	group.lidar->push_back(point);
	for (int index = 0; index < count; ++index)
	{
		auto imu = std::make_shared<sensor_msgs::msg::Imu>();
		imu->header.stamp = rclcpp::Time(int64_t(std::llround((imu_begin + index * 0.005) * 1e9)));
		imu->linear_acceleration.z = G_m_s2;
		group.imu.push_back(imu);
	}
	return group;
}

void TestPredictionAfterInitializationAndReset()
{
	ImuProcess imu;
	imu.set_gyr_cov(V3D::Constant(0.01));
	imu.set_acc_cov(V3D::Constant(0.05));
	for (const double epoch : {10.0, 1791558687.0})
	{
		imu.Reset();
		Filter filter;
		Initialize(filter);
		auto output = std::make_shared<PointCloudXYZI>();
		imu.Process(StationaryGroup(epoch, epoch + 0.08, epoch, 16), filter, output);
		const double acceleration_z = G_m_s2 + filter.get_x().grav[2];
		imu.Process(StationaryGroup(epoch + 0.08, epoch + 0.10, epoch + 0.08, 5), filter, output);
		Check(filter.get_x().pos.allFinite() && filter.get_x().vel.allFinite(), "Nonfinite first prediction");
		Check(std::abs(filter.get_x().vel[2] - acceleration_z * 0.025) < 1e-8,
			"First prediction did not start at last calibration IMU");
		Check(filter.get_x().pos.norm() < 1e-5, "Stationary first prediction made a large jump");
		auto empty = StationaryGroup(epoch + 0.10, epoch + 0.20, epoch + 0.10, 0);
		imu.Process(empty, filter, output);
		Check(output->empty(), "Missing IMU reused a previous cloud");
	}
}

void TestStationaryWindowAndRelease()
{
	const double epoch = 1791558687.0;
	auto group = StationaryGroup(epoch, epoch + 2.0, epoch, 401);
	StationaryWindow window;
	for (size_t index = 0; index < group.imu.size(); ++index)
	{
		const bool ready = window.Append(group.imu[index]);
		Check(ready == (index == 400), "Initialization did not require two seconds of IMU");
	}
	Filter filter;
	Initialize(filter);
	ImuProcess imu;
	imu.set_gyr_cov(V3D::Constant(0.01));
	imu.set_acc_cov(V3D::Constant(0.05));
	imu.InitializeStationary(window.samples, filter);
	auto held = StationaryGroup(epoch + 2, epoch + 10, epoch + 10, 1);
	imu.AdvanceStationary(held.imu.front());
	Check(filter.get_x().pos.norm() == 0.0 && filter.get_x().vel.norm() == 0.0,
		"Ground hold integrated fictitious movement");
	auto output = std::make_shared<PointCloudXYZI>();
	imu.Process(StationaryGroup(epoch + 10, epoch + 10.10, epoch + 10, 21), filter, output);
	// Compare with identical stationary initialization ending at release.
	// This isolates time spent held from upstream gravity normalization and
	// the filter's Euler integration convention.
	Filter reference;
	Initialize(reference);
	ImuProcess reference_imu;
	reference_imu.set_gyr_cov(V3D::Constant(0.01));
	reference_imu.set_acc_cov(V3D::Constant(0.05));
	reference_imu.InitializeStationary(StationaryGroup(epoch + 8, epoch + 10, epoch + 8, 401).imu, reference);
	auto reference_output = std::make_shared<PointCloudXYZI>();
	reference_imu.Process(StationaryGroup(epoch + 10, epoch + 10.10, epoch + 10, 21), reference, reference_output);
	Check((filter.get_x().pos - reference.get_x().pos).norm() < 1e-10,
		"Release integrated time spent stationary");
	Check((filter.get_x().vel - reference.get_x().vel).norm() < 1e-10,
		"Release velocity used the wrong integration boundary");
	Check(!output->empty(), "Ground initializer unexpectedly waited for more initialization clouds");
	Check(!window.Append(held.imu.front()), "A discontinuous IMU window was accepted");
	Check(window.samples.size() == 1, "Gap did not discard old calibration samples");
	window.Clear();
	for (const auto& sample : group.imu)
	{
		auto moving = std::make_shared<sensor_msgs::msg::Imu>(*sample);
		moving->angular_velocity.z = 0.1;
		Check(!window.Append(moving), "Rotating body accepted as stationary");
	}
}

void TestRecordedStationaryWindow(const char* path)
{
	std::ifstream input(path);
	Check(input.good(), "Cannot open recorded stationary IMU");
	StationaryWindow window;
	std::string line;
	int evaluated = 0;
	while (std::getline(input, line))
	{
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream row(line);
		int64_t timestamp;
		auto sample = std::make_shared<sensor_msgs::msg::Imu>();
		auto& a = sample->linear_acceleration;
		auto& w = sample->angular_velocity;
		Check(bool(row >> timestamp >> a.x >> a.y >> a.z >> w.x >> w.y >> w.z), "Malformed IMU fixture");
		sample->header.stamp = rclcpp::Time(timestamp);
		const bool ready = window.Append(sample);
		if (!window.samples.empty() && timestamp - StationaryWindow::Stamp(window.samples.front()) >= 2000000000)
		{
			Check(ready, "Recorded stationary contact falsely classified as motion");
			++evaluated;
		}
	}
	Check(evaluated > 1000, "Recorded fixture is too short");
	std::cout << "Recorded stationary windows passed: " << evaluated << std::endl;
}

int main(int argc, char** argv)
{
	TestSchmidtCovariance();
	TestPredictionAfterInitializationAndReset();
	TestStationaryWindowAndRelease();
	if (argc == 2)
	{
		TestRecordedStationaryWindow(argv[1]);
	}
	std::cout << "Startup history and Schmidt covariance checks passed" << std::endl;
	return 0;
}
