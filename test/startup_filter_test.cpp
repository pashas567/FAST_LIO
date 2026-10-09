#include <omp.h>
#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
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

int main()
{
	TestSchmidtCovariance();
	TestPredictionAfterInitializationAndReset();
	std::cout << "Startup history and Schmidt covariance checks passed" << std::endl;
	return 0;
}
