/******************************************************************
controller of articulated steering
it is a plugin or ros_controls/ros_controllers

Features:
- articulated steering kinematics
- odometry
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-04-25: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "odometry.h"
#include "speed_limiter.h"

#include <memory>
#include <string>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>
#include <urdf_parser/urdf_parser.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/TwistStamped.h>

namespace whi_articulated_steering_controller
{
    class ArticulatedSteeringController
        : public controller_interface::MultiInterfaceController<
        hardware_interface::PositionJointInterface,
        hardware_interface::VelocityJointInterface>
    {
    public:
        ArticulatedSteeringController();
        ~ArticulatedSteeringController() = default;

        /**
         * \brief Initialize controller
         * \param RobotHw      Velocity joint interface for the wheels
         * \param RootNh       Node handle at root namespace
         * \param ControllerNh Node handle inside the controller namespace
         */
        bool init(hardware_interface::RobotHW* RobotHw, ros::NodeHandle& RootNh, ros::NodeHandle& ControllerNh);

        /**
         * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
         * \param Time   Current time
         * \param Period Time since the last called to update
         */
        void update(const ros::Time& Time, const ros::Duration& Period);

        /**
         * \brief Starts controller
         * \param Time Current time
         */
        void starting(const ros::Time& Time);

        /**
         * \brief Stops controller
         * \param Time Current time
         */
        void stopping(const ros::Time& /*Time*/);

    protected:
        /**
         * \brief Brakes the wheels, i.e. sets the velocity to 0
         */
        void brake();

        /**
         * \brief Velocity command callback
         * \param Command Velocity command message (twist)
         */
        void cmdVelCallback(const geometry_msgs::Twist& Command);

        /**
         * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
         * \param RootNh                 Root node handle
         * \param RearWheelName          Name of the rear wheel joint
         * \param RotationalSteerName    Name of the rotational steer joint
         * \param LookupWheelSeparationH Whether to parse the URDF for wheel separationH
         * \param LookupWheelRadius      Whether to parse the URDF for wheel radius
         */
        bool setOdomParamsFromUrdf(ros::NodeHandle& RootNh,
            const std::string& RearWheelName, const std::string& FrontWheelName, const std::string& RotationalSteerName,
            bool LookupWheelSeparation, bool LookupWheelRadius);

        /**
         * \brief Sets the odometry publishing fields
         * \param RootNh       Root node handle
         * \param ControllerNh Node handle inside the controller namespace
         */
        void setOdomPubFields(ros::NodeHandle& RootNh, ros::NodeHandle& ControllerNh);

        geometry_msgs::Point applyRotationXy(const geometry_msgs::Point& Src,
            const geometry_msgs::Point& Center, double Theta);
        void publishDynamicFootprint(double SteerPos);

    protected:
        static bool getWheelRadius(const urdf::LinkConstSharedPtr& WheelLink, double& WheelRadius);
        static bool isCylinder(const urdf::LinkConstSharedPtr& Link);
        static geometry_msgs::Polygon toMsg(const std::vector<geometry_msgs::Point>& Points);

    private:
        std::string name_;

        // odometry related:
        ros::Duration publish_period_;
        ros::Time last_state_publish_time_;
        bool open_loop_{ false };

        // hardware handles:
        hardware_interface::JointHandle rear_wheel_joint_;
        hardware_interface::JointHandle rotational_steer_joint_;

        // velocity command related:
        struct Commands
        {
            double lin{ 0.0 };
            double ang{ 0.0 };
            ros::Time stamp{ 0.0 };
        };
        realtime_tools::RealtimeBuffer<Commands> command_;
        Commands command_struct_;
        ros::Subscriber sub_command_;

        // odometry related:
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_{ nullptr };
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_{ nullptr };
        std::unique_ptr<Odometry> odometry_{ nullptr };
        // publish executed commands
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

        // wheel separation, wrt the midpoint of the wheel width:
        double wheel_separation_rear_{ 0.15 };
        double wheel_separation_front_{ 0.15 };

        // wheel radius (assuming it's the same for the left and right wheels):
        double wheel_radius_{ 0.0325 };

        // wheel separation and radius calibration multipliers:
        double wheel_separation_rear_multiplier_{ 1.0 };
        double wheel_separation_front_multiplier_{ 1.0 };
        double wheel_radius_multiplier_{ 1.0 };
        double steer_pos_multiplier_{ 1.0 };

        // timeout to consider cmd_vel commands old:
        double cmd_vel_timeout_{ 0.5 };

        // whether to allow multiple publishers on cmd_vel topic or not:
        bool allow_multiple_cmd_vel_publishers_{ true };

        // frame to use for the robot base:
        std::string base_frame_id_{ "base_link" };

        // frame to use for odometry and odom tf:
        std::string odom_frame_id_{ "odom" };

        // whether to publish odometry to tf or not:
        bool enable_odom_tf_{ true };

        // publish executed velocity:
        bool publish_cmd_{ false };

        // speed limiters:
        Commands last1_cmd_;
        Commands last0_cmd_;
        whi_kinematic_controller::SpeedLimiter limiter_lin_;
        whi_kinematic_controller::SpeedLimiter limiter_ang_;

        std::vector<geometry_msgs::Point> foot_print_trailer_;
        std::vector<geometry_msgs::Point> foot_print_joint_;
        std::vector<geometry_msgs::Point> foot_print_tractor_;
        geometry_msgs::Point pivot_axis_;
        std::vector<geometry_msgs::Point> foot_print_poly_;
        std::unique_ptr<ros::Publisher> pub_footprint_{ nullptr };
    };
} // namespace whi_articulated_steering_controller
