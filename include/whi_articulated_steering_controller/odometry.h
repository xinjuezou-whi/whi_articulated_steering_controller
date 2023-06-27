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
#pragma once
#include <ros/time.h>
#include <functional>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

namespace whi_articulated_steering_controller
{
    namespace bacc = boost::accumulators;

    /**
     * \brief The Odometry class handles odometry readings
     * (2D pose and velocity with related timestamp)
     */
    class Odometry
    {
    public:
        /// integration function, used to integrate the odometry:
        using IntegrationFunction = std::function<void(double, double)>;

        /**
         * \brief Constructor
         * Timestamp will get the current time value
         * Value will be set to zero
         * \param VelocityRollingWindowSize Rolling window size used to compute the velocity mean
         */
        Odometry() = delete;
        Odometry(double WheelSeparationRear, double WheelSeparationFront,
            double WheelRadius, size_t VelocityRollingWindowSize = 10);
        ~Odometry() = default;

        /**
         * \brief Initialize the odometry
         * \param Time Current time
         */
        void init(const ros::Time& Time);

        /**
         * \brief Updates the odometry class with latest wheels position
         * \param RearWheelPos       Rear wheel position [rad]
         * \param RotationalSteerPos Front Steer position [rad]
         * \param Time               Current time
         * \return true if the odometry is actually updated
         */
        bool update(double RearWheelPos, double RotationalSteerPos, const ros::Time& Time);

        /**
         * \brief Updates the odometry class with latest velocity command
         * \param Linear  Linear velocity [m/s]
         * \param Angular Angular velocity [rad/s]
         * \param Time    Current time
         */
        void updateOpenLoop(double Linear, double Angular, const ros::Time& Time);

        /**
         * \brief heading getter
         * \return heading [rad]
         */
        double getHeading() const
        {
            return heading_;
        }

        /**
         * \brief x position getter
         * \return x position [m]
         */
        double getX() const
        {
            return x_;
        }

        /**
         * \brief y position getter
         * \return y position [m]
         */
        double getY() const
        {
            return y_;
        }

        /**
         * \brief linear velocity getter
         * \return linear velocity [m/s]
         */
        double getLinear() const
        {
            return linear_;
        }

        /**
         * \brief angular velocity getter
         * \return angular velocity [rad/s]
         */
        double getAngular() const
        {
            return angular_;
        }

    private:
        /// rolling mean accumulator and window:
        using RollingMeanAcc = bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean>>;
        using RollingWindow = bacc::tag::rolling_window;

        /**
         * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
         * \param Linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
         * \param Angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
         */
        void integrateRungeKutta2(double Linear, double Angular);

        /**
         * \brief Integrates the velocities (linear and angular) using exact method
         * \param Linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
         * \param Angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
         */
        void integrateExact(double Linear, double Angular);

    private:
        // current timestamp:
        ros::Time time_stamp_{ 0.0 };

        // current pose:
        double x_{ 0.0 };        //   [m]
        double y_{ 0.0 };        //   [m]
        double heading_{ 0.0 };  // [rad]

        // current velocity:
        double linear_{ 0.0 };  //   [m/s]
        double angular_{ 0.0 }; // [rad/s]

        // wheel kinematic parameters [m]:
        double wheel_separation_rear_{ 0.15 };
        double wheel_separation_front_{ 0.15 };
        double wheel_radius_{ 0.0325 };

        // previous steer position
        double old_steer_pos_{ 0.0 };
        // previous wheel position/state [rad]:
        double rear_wheel_old_pos_{ 0.0 };

        // rolling mean accumulators for the linar and angular velocities:
        size_t velocity_rolling_window_size_{ 10 };
        RollingMeanAcc linear_acc_;
        RollingMeanAcc angular_acc_;

        // integration funcion, used to integrate the odometry:
        IntegrationFunction integrate_fun_{ nullptr };
    };
} // namespace whi_articulated_steering_controller
