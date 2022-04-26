/******************************************************************
odometry of articulated steering

Features:
- articulated steering kinematics
- odometry
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-04-26: Initial version
2022-xx-xx: xxx
******************************************************************/
#include "whi_articulated_steering_controller/odometry.h"

namespace whi_articulated_steering_controller
{
	namespace bacc = boost::accumulators;

	Odometry::Odometry(double WheelSeparationH, double WheelRadius, size_t VelocityRollingWindowSize /*= 10*/)
		: time_stamp_(0.0)
		, x_(0.0)
		, y_(0.0)
		, heading_(0.0)
		, linear_(0.0)
		, angular_(0.0)
		, wheel_separation_h_(WheelSeparationH)
		, wheel_radius_(WheelRadius)
		, rear_wheel_old_pos_(0.0)
		, velocity_rolling_window_size_(VelocityRollingWindowSize)
		, linear_acc_(RollingWindow::window_size = VelocityRollingWindowSize)
		, angular_acc_(RollingWindow::window_size = VelocityRollingWindowSize)
		, integrate_fun_(std::bind(&Odometry::integrateExact, this, std::placeholders::_1, std::placeholders::_2))
	{
	}

	void Odometry::init(const ros::Time& Time)
	{
		// reset timestamp:
		time_stamp_ = Time;
	}

	bool Odometry::update(double RearWheelPos, double RotationalSteerPos, const ros::Time& Time)
	{
		/// get current wheel joint positions:
		const double rearWheelCurPos = RearWheelPos * wheel_radius_;

		/// estimate velocity of wheels using old and current position:
		const double rearWheelEstVel = rearWheelCurPos - rear_wheel_old_pos_;

		/// update old position with current:
		rear_wheel_old_pos_ = rearWheelCurPos;

		/// compute linear and angular diff:
		const double linear = rearWheelEstVel;
		const double angular = tan(RotationalSteerPos) * linear / wheel_separation_h_;

		/// integrate odometry:
		integrate_fun_(linear, angular);

		/// we cannot estimate the speed with very small time intervals:
		const double dt = (Time - time_stamp_).toSec();
		if (dt < 0.0001)
		{
			return false; // interval too small to integrate with
		}

		time_stamp_ = Time;

		/// estimate speeds using a rolling mean to filter them out:
		linear_acc_(linear / dt);
		angular_acc_(angular / dt);

		linear_ = bacc::rolling_mean(linear_acc_);
		angular_ = bacc::rolling_mean(angular_acc_);

		return true;
	}

	void Odometry::updateOpenLoop(double Linear, double Angular, const ros::Time& Time)
	{
		/// save last linear and angular velocity:
		linear_ = Linear;
		angular_ = Angular;

		/// integrate odometry:
		const double dt = (Time - time_stamp_).toSec();
		time_stamp_ = Time;
		integrate_fun_(Linear * dt, Angular * dt);
	}

	void Odometry::integrateRungeKutta2(double Linear, double Angular)
	{
		const double direction = heading_ + Angular * 0.5;

		// runge-Kutta 2nd order integration:
		x_ += Linear * cos(direction);
		y_ += Linear * sin(direction);
		heading_ += Angular;
	}

	void Odometry::integrateExact(double Linear, double Angular)
	{
		if (fabs(Angular) < 1e-6)
		{
			integrateRungeKutta2(Linear, Angular);
		}
		else
		{
			// exact integration (should solve problems when angular is zero):
			const double headingOld = heading_;
			const double r = Linear / Angular;
			heading_ += Angular;
			x_ += r * (sin(heading_) - sin(headingOld));
			y_ += -r * (cos(heading_) - cos(headingOld));
		}
	}
} // namespace whi_articulated_steering_controller
