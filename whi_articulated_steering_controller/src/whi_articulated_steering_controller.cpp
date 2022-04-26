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

******************************************************************/
#include "whi_articulated_steering_controller/whi_articulated_steering_controller.h"

#include <memory>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>

namespace articulated_steering_controller
{
    bool ArticulatedSteeringController::init(hardware_interface::RobotHW* RobotHw,
        ros::NodeHandle& RootNh,
        ros::NodeHandle& ControllerNh)
    {
        using VelIface = hardware_interface::VelocityJointInterface;
        using PosIface = hardware_interface::PositionJointInterface;
        using StateIface = hardware_interface::JointStateInterface;

        // get multiple types of hardware_interface
        VelIface* velJointIf = RobotHw->get<VelIface>(); // vel for wheels
        PosIface* posJointIf = RobotHw->get<PosIface>(); // pos for steers

        const std::string completeNamespace = ControllerNh.getNamespace();

        std::size_t id = completeNamespace.find_last_of("/");
        name_ = completeNamespace.substr(id + 1);

        // single rear wheel joint
        std::string rearWheelName;
        ControllerNh.param("rear_wheel", rearWheelName, std::string("joint_rear_wheel"));

        // single rotational steer joint
        std::string rotationalSteerName;
        ControllerNh.param("front_steer", rotationalSteerName, std::string("joint_rotational_steer"));


        // odometry related:
        double publishRate = 0.0;
        ControllerNh.param("publish_rate", publishRate, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "controller state will be published at " << publishRate << "Hz.");
        publish_period_ = ros::Duration(1.0 / publishRate);

        ControllerNh.param("open_loop", open_loop_, false);

        ControllerNh.param("wheel_separation_h_multiplier", wheel_separation_h_multiplier_, 1.0);
        ROS_INFO_STREAM_NAMED(name_, "wheel separation height will be multiplied by " << wheel_separation_h_multiplier_ << ".");

        ControllerNh.param("wheel_radius_multiplier", wheel_radius_multiplier_, 1.0);
        ROS_INFO_STREAM_NAMED(name_, "wheel radius will be multiplied by " << wheel_radius_multiplier_ << ".");

        ControllerNh.param("steer_pos_multiplier", steer_pos_multiplier_, 1.0);
        ROS_INFO_STREAM_NAMED(name_, "steer pos will be multiplied by " << steer_pos_multiplier_ << ".");

        int velocityRollingWindowSize = 10;
        ControllerNh.param("velocity_rolling_window_size", velocityRollingWindowSize, 10);
        ROS_INFO_STREAM_NAMED(name_, "velocity rolling window size of " << velocityRollingWindowSize << ".");

        // twist command related:
        ControllerNh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
        ROS_INFO_STREAM_NAMED(name_, "velocity commands will be considered old if they are older than " << cmd_vel_timeout_ << "s.");

        ControllerNh.param("allow_multiple_cmd_vel_publishers", allow_multiple_cmd_vel_publishers_, true);
        ROS_INFO_STREAM_NAMED(name_, "allow mutiple cmd_vel publishers is " << (allow_multiple_cmd_vel_publishers_ ? "enabled" : "disabled"));

        ControllerNh.param("base_frame_id", base_frame_id_, std::string("base_link"));
        ROS_INFO_STREAM_NAMED(name_, "base frame_id set to " << base_frame_id_);

        ControllerNh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
        ROS_INFO_STREAM_NAMED(name_, "odometry frame_id set to " << odom_frame_id_);

        ControllerNh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

        // velocity and acceleration limits:
        ControllerNh.param("linear/x/has_velocity_limits", limiter_lin_.has_velocity_limits_, false);
        ControllerNh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits_, false);
        ControllerNh.param("linear/x/has_jerk_limits", limiter_lin_.has_jerk_limits_, false);
        ControllerNh.param("linear/x/max_velocity", limiter_lin_.max_velocity_, 0.2);
        ControllerNh.param("linear/x/min_velocity", limiter_lin_.min_velocity_, -0.2);
        ControllerNh.param("linear/x/max_acceleration", limiter_lin_.max_acceleration_, 0.5);
        ControllerNh.param("linear/x/min_acceleration", limiter_lin_.min_acceleration_, -0.5);
        ControllerNh.param("linear/x/max_jerk", limiter_lin_.max_jerk_, 0.5);
        ControllerNh.param("linear/x/min_jerk", limiter_lin_.min_jerk_, -0.5);

        ControllerNh.param("angular/z/has_velocity_limits", limiter_ang_.has_velocity_limits_, false);
        ControllerNh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits_, false);
        ControllerNh.param("angular/z/has_jerk_limits", limiter_ang_.has_jerk_limits_, false);
        ControllerNh.param("angular/z/max_velocity", limiter_ang_.max_velocity_, 0.5);
        ControllerNh.param("angular/z/min_velocity", limiter_ang_.min_velocity_, -0.5);
        ControllerNh.param("angular/z/max_acceleration", limiter_ang_.max_acceleration_, 0.5);
        ControllerNh.param("angular/z/min_acceleration", limiter_ang_.min_acceleration_, -0.5);
        ControllerNh.param("angular/z/max_jerk", limiter_ang_.max_jerk_, 0.5);
        ControllerNh.param("angular/z/min_jerk", limiter_ang_.min_jerk_, -0.5);

        // if either parameter is not available, we need to look up the value in the URDF
        bool lookupWheelSeparationH = !ControllerNh.getParam("wheel_separation_h", wheel_separation_h_);
        bool lookupWheelRadius = !ControllerNh.getParam("wheel_radius", wheel_radius_);

        if (!setOdomParamsFromUrdf(RootNh, rearWheelName, rotationalSteerName, lookupWheelSeparationH, lookupWheelRadius))
        {
            return false;
        }

        // regardless of how we got the separation and radius, use them to set the odometry parameters
        const double wheelSeparationH = wheel_separation_h_multiplier_ * wheel_separation_h_;
        const double wheelRadius = wheel_radius_multiplier_ * wheel_radius_;
        odometry_ = std::make_unique<Odometry>(wheelSeparationH, wheelRadius, velocityRollingWindowSize);
        ROS_INFO_STREAM_NAMED(name_, "odometry params : wheel separation height " << wheelSeparationH << ", wheel radius " << wheelRadius);

        setOdomPubFields(RootNh, ControllerNh);

        // rear wheel
        ROS_INFO_STREAM_NAMED(name_, "adding the rear wheel with joint name: " << rearWheelName);
        rear_wheel_joint_ = velJointIf->getHandle(rearWheelName); // throws on failure
        // front steer
        ROS_INFO_STREAM_NAMED(name_, "adding the front steer with joint name: " << rotationalSteerName);
        rotational_steer_joint_ = posJointIf->getHandle(rotationalSteerName); // throws on failure
        ROS_INFO_STREAM_NAMED(name_, "adding the subscriber: cmd_vel");
        sub_command_ = ControllerNh.subscribe("cmd_vel", 1, &ArticulatedSteeringController::cmdVelCallback, this);
        ROS_INFO_STREAM_NAMED(name_, "finished controller initialization");

        return true;
    }

    void ArticulatedSteeringController::update(const ros::Time& Time, const ros::Duration& Period)
    {
        // calculate and publish the odometry
        if (open_loop_)
        {
            odometry_->updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, Time);
        }
        else
        {
            double wheelPos = rear_wheel_joint_.getPosition();
            double steerPos = rotational_steer_joint_.getPosition();

            if (std::isnan(wheelPos) || std::isnan(steerPos))
            {
                return;
            }

            // estimate linear and angular velocity using joint information
            steerPos = steerPos * steer_pos_multiplier_;
            odometry_->update(wheelPos, steerPos, Time);
        }

        // publish odometry message
        if (last_state_publish_time_ + publish_period_ < Time)
        {
            last_state_publish_time_ += publish_period_;
            // compute and store orientation info
            const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_->getHeading()));

            // populate odom message and publish
            if (odom_pub_->trylock())
            {
                odom_pub_->msg_.header.stamp = Time;
                odom_pub_->msg_.pose.pose.position.x = odometry_->getX();
                odom_pub_->msg_.pose.pose.position.y = odometry_->getY();
                odom_pub_->msg_.pose.pose.orientation = orientation;
                odom_pub_->msg_.twist.twist.linear.x = odometry_->getLinear();
                odom_pub_->msg_.twist.twist.angular.z = odometry_->getAngular();
                odom_pub_->unlockAndPublish();
            }

            // publish tf /odom frame
            if (enable_odom_tf_ && tf_odom_pub_->trylock())
            {
                geometry_msgs::TransformStamped& odomFrame = tf_odom_pub_->msg_.transforms[0];
                odomFrame.header.stamp = Time;
                odomFrame.transform.translation.x = odometry_->getX();
                odomFrame.transform.translation.y = odometry_->getY();
                odomFrame.transform.rotation = orientation;
                tf_odom_pub_->unlockAndPublish();
            }
        }

        /// move robot
        // retreive current velocity command and time step:
        Commands currCmd = *(command_.readFromRT());
        const double dt = (Time - currCmd.stamp).toSec();

        // brake if cmd_vel has timeout:
        if (dt > cmd_vel_timeout_)
        {
            currCmd.lin = 0.0;
            currCmd.ang = 0.0;
        }

        // limit velocities and accelerations:
        const double cmdDt(Period.toSec());

        limiter_lin_.limit(currCmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmdDt);
        limiter_ang_.limit(currCmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmdDt);

        last1_cmd_ = last0_cmd_;
        last0_cmd_ = currCmd;

        // set Command
        const double wheelVel = currCmd.lin / wheel_radius_; // omega = linear_vel / radius
        rear_wheel_joint_.setCommand(wheelVel);
        rotational_steer_joint_.setCommand(currCmd.ang);
    }

    void ArticulatedSteeringController::starting(const ros::Time& Time)
    {
        brake();

        // register starting time used to keep fixed rate
        last_state_publish_time_ = Time;

        odometry_->init(Time);
    }

    void ArticulatedSteeringController::stopping(const ros::Time& /*Time*/)
    {
        brake();
    }

    void ArticulatedSteeringController::brake()
    {
        const double steerPos = 0.0;
        const double wheelVel = 0.0;

        rear_wheel_joint_.setCommand(steerPos);
        rotational_steer_joint_.setCommand(wheelVel);
    }

    void ArticulatedSteeringController::cmdVelCallback(const geometry_msgs::Twist& Command)
    {
        if (isRunning())
        {
            // check that we don't have multiple publishers on the command topic
            if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
            {
                ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "detected " << sub_command_.getNumPublishers()
                    << " publishers. only 1 publisher is allowed. going to brake");
                
                brake();
                return;
            }

            command_struct_.ang = Command.angular.z;
            command_struct_.lin = Command.linear.x;
            command_struct_.stamp = ros::Time::now();
            command_.writeFromNonRT(command_struct_);
            ROS_DEBUG_STREAM_NAMED(name_,
                "added values to command. "
                << "Angular: " << command_struct_.ang << ", "
                << "linear: " << command_struct_.lin << ", "
                << "Stamp: " << command_struct_.stamp);
        }
        else
        {
            ROS_ERROR_NAMED(name_, "can't accept new commands. controller is not running.");
        }
    }

    bool ArticulatedSteeringController::setOdomParamsFromUrdf(ros::NodeHandle& RootNh,
        const std::string RearWheelName, const std::string RotationalSteerName,
        bool LookupWheelSeparationH, bool LookupWheelRadius)
    {
        if (!(LookupWheelSeparationH || LookupWheelRadius))
        {
            // short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
            return true;
        }

        // parse robot description
        const std::string modelParamName = "robot_description";
        bool res = RootNh.hasParam(modelParamName);
        std::string robotModelStr = "";
        if (!res || !RootNh.getParam(modelParamName, robotModelStr))
        {
            ROS_ERROR_NAMED(name_, "robot descripion couldn't be retrieved from param server");
            return false;
        }

        urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robotModelStr));

        urdf::JointConstSharedPtr rearWheelJoint(model->getJoint(RearWheelName));
        urdf::JointConstSharedPtr rotationalSteerJoint(model->getJoint(RotationalSteerName));

        if (LookupWheelSeparationH)
        {
            // get wheel separation
            if (!rearWheelJoint)
            {
                ROS_ERROR_STREAM_NAMED(name_, RearWheelName << " couldn't be retrieved from model description");

                return false;
            }

            if (!rotationalSteerJoint)
            {
                ROS_ERROR_STREAM_NAMED(name_, RotationalSteerName << " couldn't be retrieved from model description");

                return false;
            }

            ROS_INFO_STREAM("rear wheel to origin: "
                << rearWheelJoint->parent_to_joint_origin_transform.position.x << ","
                << rearWheelJoint->parent_to_joint_origin_transform.position.y << ", "
                << rearWheelJoint->parent_to_joint_origin_transform.position.z);

            ROS_INFO_STREAM("front steer to origin: "
                << rotationalSteerJoint->parent_to_joint_origin_transform.position.x << ","
                << rotationalSteerJoint->parent_to_joint_origin_transform.position.y << ", "
                << rotationalSteerJoint->parent_to_joint_origin_transform.position.z);

            wheel_separation_h_ = fabs(rearWheelJoint->parent_to_joint_origin_transform.position.x -
                rotationalSteerJoint->parent_to_joint_origin_transform.position.x);

            ROS_INFO_STREAM("calculated wheel_separation_h: " << wheel_separation_h_);
        }

        if (LookupWheelRadius)
        {
            // get wheel radius
            if (!getWheelRadius(model->getLink(rearWheelJoint->child_link_name), wheel_radius_))
            {
                ROS_ERROR_STREAM_NAMED(name_, "couldn't retrieve " << RearWheelName << " wheel radius");

                return false;
            }
            ROS_INFO_STREAM("retrieved wheel_radius: " << wheel_radius_);
        }

        return true;
    }

    void ArticulatedSteeringController::setOdomPubFields(ros::NodeHandle& RootNh, ros::NodeHandle& ControllerNh)
    {
        // get and check params for covariances
        XmlRpc::XmlRpcValue poseCovList;
        ControllerNh.getParam("pose_covariance_diagonal", poseCovList);
        ROS_ASSERT(poseCovList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(poseCovList.size() == 6);

        for (const auto& it : poseCovList)
        {
            ROS_ASSERT(it.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        }

        XmlRpc::XmlRpcValue twistCovList;
        ControllerNh.getParam("twist_covariance_diagonal", twistCovList);
        ROS_ASSERT(twistCovList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twistCovList.size() == 6);
        for (const auto& it : twistCovList)
        {
            ROS_ASSERT(it.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        }

        // setup odometry realtime publisher + odom message constant fields
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(ControllerNh, "odom", 100));
        odom_pub_->msg_.header.frame_id = odom_frame_id_;
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance =
        {
            static_cast<double>(poseCovList[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(poseCovList[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(poseCovList[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(poseCovList[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(poseCovList[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(poseCovList[5])
        };
        odom_pub_->msg_.twist.twist.linear.y = 0;
        odom_pub_->msg_.twist.twist.linear.z = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance =
        {
            static_cast<double>(twistCovList[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(twistCovList[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(twistCovList[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(twistCovList[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(twistCovList[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(twistCovList[5])
        };
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(RootNh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    }

    bool ArticulatedSteeringController::getWheelRadius(const urdf::LinkConstSharedPtr& WheelLink, double& WheelRadius)
    {
        if (!isCylinder(WheelLink))
        {
            ROS_ERROR_STREAM("wheel link " << WheelLink->name << " is NOT modeled as a cylinder!");

            return false;
        }

        WheelRadius = (static_cast<urdf::Cylinder*>(WheelLink->collision->geometry.get()))->radius;

        return true;
    }

    bool ArticulatedSteeringController::isCylinder(const urdf::LinkConstSharedPtr& Link)
    {
        if (!Link)
        {
            ROS_ERROR("link pointer is null");

            return false;
        }

        if (!Link->collision)
        {
            ROS_ERROR_STREAM("Link " << Link->name << " does not have collision description. add collision description for link to urdf");

            return false;
        }

        if (!Link->collision->geometry)
        {
            ROS_ERROR_STREAM("link " << Link->name << " does not have collision geometry description. add collision geometry description for link to urdf");
            
            return false;
        }

        if (Link->collision->geometry->type != urdf::Geometry::CYLINDER)
        {
            ROS_ERROR_STREAM("link " << Link->name << " does not have cylinder geometry");

            return false;
        }

        return true;
    }

    PLUGINLIB_EXPORT_CLASS(ArticulatedSteeringController, controller_interface::ControllerBase)
} // namespace articulated_steering_controller
