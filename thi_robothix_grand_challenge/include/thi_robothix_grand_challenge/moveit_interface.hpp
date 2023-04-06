#ifndef MOVEIT_INTERFACE_HPP_
#define MOVEIT_INTERFACE_HPP_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <memory>
#include "ros/ros.h"
#include <moveit/robot_state/robot_state.h>
#include <stdexcept>

class MoveItArmInterface
{
public:
    MoveItArmInterface(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi, const double planning_time, const double max_vel_scale_factor, const double max_acc_scale_factor) : mgi_(mgi)
    {
        mgi_->setPlanningTime(planning_time);
        mgi_->setMaxVelocityScalingFactor(max_vel_scale_factor);
        mgi_->setMaxAccelerationScalingFactor(max_acc_scale_factor);
    };

    bool moveToFramePTP(std::string frame_id, double z_offset_m)
    {
        bool success = false;

        geometry_msgs::TransformStamped transformStamped;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Duration(1.0).sleep();
        try
        {
            transformStamped = tfBuffer.lookupTransform("panda_link0", frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();

            return success;
        }

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "panda_link0";

        try
        {
            geometry_msgs::TransformStamped transform_hand_to_tcp = tfBuffer.lookupTransform("panda_hand", "panda_hand_tcp", ros::Time(0));

            target_pose.pose.position.x = transformStamped.transform.translation.x + transform_hand_to_tcp.transform.translation.x;
            target_pose.pose.position.y = transformStamped.transform.translation.y + transform_hand_to_tcp.transform.translation.y;
            target_pose.pose.position.z = transformStamped.transform.translation.z + transform_hand_to_tcp.transform.translation.z + z_offset_m;

            target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
            target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
            target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();

            target_pose.pose.position.x = transformStamped.transform.translation.x;
            target_pose.pose.position.y = transformStamped.transform.translation.y;
            target_pose.pose.position.z = transformStamped.transform.translation.z;

            target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
            target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
            target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
        }

        mgi_->setPoseTarget(target_pose);

        success = (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
            mgi_->move();
        return success;
    }

    bool moveToFrameLinear(std::string frame_id, double z_offset_m)
    {
        bool success = false;

        std::vector<geometry_msgs::Pose> waypoints;

        geometry_msgs::Pose target_pose;
        geometry_msgs::TransformStamped transformStamped;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Duration(1.0).sleep();

        try
        {
            // must be transformed from world and not
            // from link0, because Pose is beign used here and not PoseStamped
            transformStamped = tfBuffer.lookupTransform("world", frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();

            return success;
        }

        // tryTCPTransform(&target_pose,&tfBuffer);

        try
        {
            geometry_msgs::TransformStamped transform_hand_to_tcp = tfBuffer.lookupTransform("panda_hand", "panda_hand_tcp", ros::Time(0));

            target_pose.position.x = transformStamped.transform.translation.x + transform_hand_to_tcp.transform.translation.x;
            target_pose.position.y = transformStamped.transform.translation.y + transform_hand_to_tcp.transform.translation.y;
            target_pose.position.z = transformStamped.transform.translation.z + transform_hand_to_tcp.transform.translation.z + z_offset_m;

            target_pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.orientation.x = transformStamped.transform.rotation.x;
            target_pose.orientation.y = transformStamped.transform.rotation.y;
            target_pose.orientation.z = transformStamped.transform.rotation.z;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();

            target_pose.position.x = transformStamped.transform.translation.x;
            target_pose.position.y = transformStamped.transform.translation.y;
            target_pose.position.z = transformStamped.transform.translation.z;

            target_pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.orientation.x = transformStamped.transform.rotation.x;
            target_pose.orientation.y = transformStamped.transform.rotation.y;
            target_pose.orientation.z = transformStamped.transform.rotation.z;
        }

        // Add Target Pose to waypoints
        waypoints.push_back(target_pose);

        /*We want the Cartesian path to be interpolated at a resolution of 1 cm which is
        why we will specify 0.01 as the max step in Cartesian translation.
        We will specify the jump threshold as 0.0, effectively disabling it.
        !!!! Warning - disabling the jump threshold while operating real hardware can cause large
        unpredictable motions of redundant joints and could be a safety issue!!!*/

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = mgi_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        success = (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
            mgi_->execute(trajectory);

        return success;
    }

    bool approachFrame(std::string frame_id, double z_offset_mm, std::string motion_type)
    {
        double z_offset_m = z_offset_mm / 1000; // going from mm -> m

        if (motion_type.compare(lin))
        {
            return moveToFrameLinear(frame_id, z_offset_m);
        }
        else if (motion_type.compare(ptp))
        {
            return moveToFramePTP(frame_id, z_offset_m);
        }
        else
        {
            std::string error = std::string("Non defined motion type:") + motion_type;
            throw std::runtime_error(error);
            return false;
        }
    }



    bool moveToHome()
    {
        // Home Position for Franka Emika
        return moveToFramePTP("home");
    }

    bool moveCircular()
    {
        bool success = false;

        return success;
    }

    bool moveToFrameLinear(std::string frame_id)
    {
        return moveToFrameLinear(frame_id, 0);
    }

    bool moveToFramePTP(std::string frame_id)
    {
    return moveToFramePTP(frame_id, 0);
    }

    public:
    const std::string lin = "lin";
    const std::string ptp = "ptp";

    private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_;
};

class MoveItGripperInterface
{
public:
    MoveItGripperInterface(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi, const double planning_time, const double max_vel_scale_factor, const double max_acc_scale_factor) : mgi_(mgi)
    {
        mgi_->setPlanningTime(planning_time);
        mgi_->setMaxVelocityScalingFactor(max_vel_scale_factor);
        mgi_->setMaxAccelerationScalingFactor(max_acc_scale_factor);
    };

    bool closeGripper()
    {
        mgi_->setJointValueTarget(mgi_->getNamedTargetValues("close"));

        bool success = (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            mgi_->move();

        return success;
    }

    bool openGripper()
    {
        mgi_->setJointValueTarget(mgi_->getNamedTargetValues("open"));

        bool success = (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            mgi_->move();

        return success;
    }

    bool setGripperWidth(double width)
    {
        std::vector<double> finger_width;
        finger_width.resize(2);
        finger_width[0] = width;
        finger_width[1] = width;

        mgi_->setJointValueTarget(finger_width);

        const bool success = (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
            mgi_->move();

        return success;
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_;
};

#endif