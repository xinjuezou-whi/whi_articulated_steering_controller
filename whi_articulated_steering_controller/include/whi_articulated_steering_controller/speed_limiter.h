/******************************************************************
speed limiter for kinematic controller

Features:
- speed limiter
- xxx

Original by Enrique Fern¨¢ndez from PAL Robotics, S.L.
Refactored by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#pragma once
#include "whi_articulated_steering_controller/speed_limiter.h"

namespace whi_kinematic_controller
{
    class SpeedLimiter
    {
    public:
        SpeedLimiter() = default;

        /**
         * \brief Limit the velocity and acceleration
         * \param [in, out] V  Velocity [m/s]
         * \param [in]      V0 Previous velocity to V  [m/s]
         * \param [in]      V1 Previous velocity to V0 [m/s]
         * \param [in]      Dt Time step [s]
         * \return Limiting factor (1.0 if none)
         */
        double limit(double& V, double V0, double V1, double Dt);

        /**
         * \brief Limit the velocity
         * \param [in, out] V Velocity [m/s]
         * \return Limiting factor (1.0 if none)
         */
        double limitVelocity(double& V);

        /**
         * \brief Limit the acceleration
         * \param [in, out] V  Velocity [m/s]
         * \param [in]      V0 Previous velocity [m/s]
         * \param [in]      Dt Time step [s]
         * \return Limiting factor (1.0 if none)
         */
        double limitAcceleration(double& V, double V0, double Dt);

        /**
         * \brief Limit the jerk
         * \param [in, out] V  Velocity [m/s]
         * \param [in]      V0 Previous velocity to v  [m/s]
         * \param [in]      V1 Previous velocity to v0 [m/s]
         * \param [in]      Dt Time step [s]
         * \return Limiting factor (1.0 if none)
         * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
         */
        double limitJerk(double& V, double V0, double V1, double Dt);

    public:
        // enable/disable velocity/acceleration/jerk limits
        bool has_velocity_limits_{ false };
        bool has_acceleration_limits_{ false };
        bool has_jerk_limits_{ false };

        // velocity limits
        double min_velocity_{ 0.0 };
        double max_velocity_{ 0.0 };

        // acceleration limits
        double min_acceleration_{ 0.0 };
        double max_acceleration_{ 0.0 };

        // jerk limits
        double min_jerk_{ 0.0 };
        double max_jerk_{ 0.0 };
    };
} // namespace whi_kinematic_controller
