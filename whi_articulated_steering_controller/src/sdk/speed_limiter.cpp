/******************************************************************
speed limiter for kinematic controller

Features:
- speed limiter
- xxx

Original by Enrique Fern¨¢ndez from PAL Robotics, S.L.
Refactored by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-04-26: Initial version
2022-xx-xx: xxx
******************************************************************/
#include "whi_articulated_steering_controller/speed_limiter.h"

#include <algorithm>

namespace whi_kinematic_controller
{
    template<typename T>
    T clamp(T X, T Min, T Max)
    {
        return std::min(std::max(Min, X), Max);
    }

    double SpeedLimiter::limit(double& V, double V0, double V1, double Dt)
    {
        const double tmp = V;

        limitJerk(V, V0, V1, Dt);
        limitAcceleration(V, V0, Dt);
        limitVelocity(V);

        return tmp != 0.0 ? V / tmp : 1.0;
    }

    double SpeedLimiter::limitVelocity(double& V)
    {
        const double tmp = V;

        if (has_velocity_limits_)
        {
            V = clamp(V, min_velocity_, max_velocity_);
        }

        return tmp != 0.0 ? V / tmp : 1.0;
    }

    double SpeedLimiter::limitAcceleration(double& V, double V0, double Dt)
    {
        const double tmp = V;

        if (has_acceleration_limits_)
        {
            const double dvMin = min_acceleration_ * Dt;
            const double dvMax = max_acceleration_ * Dt;

            const double dv = clamp(V - V0, dvMin, dvMax);

            V = V0 + dv;
        }

        return tmp != 0.0 ? V / tmp : 1.0;
    }

    double SpeedLimiter::limitJerk(double& V, double V0, double V1, double Dt)
    {
        const double tmp = V;

        if (has_jerk_limits_)
        {
            const double dv = V - V0;
            const double dv0 = V0 - V1;

            const double dt2 = 2. * Dt * Dt;

            const double daMin = min_jerk_ * dt2;
            const double daMax = max_jerk_ * dt2;

            const double da = clamp(dv - dv0, daMin, daMax);

            V = V0 + dv0 + da;
        }

        return tmp != 0.0 ? V / tmp : 1.0;
    }
} // namespace whi_kinematic_controller
