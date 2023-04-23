#ifndef MOVEIT_INTERFACE_HPP_
#define MOVEIT_INTERFACE_HPP_

#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include <memory>
#include <string>
#include <stdexcept>

#define DEBUG_VISUALIZATION false

class MoveItArmInterface
{
  public:
    MoveItArmInterface(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi, const double planning_time, const double max_vel_scale_factor, const double max_acc_scale_factor, std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools) : mgi_(mgi), visual_tools_(visual_tools)
    {
        mgi_->setPlanningTime(planning_time);
        mgi_->setMaxVelocityScalingFactor(max_vel_scale_factor);
        mgi_->setMaxAccelerationScalingFactor(max_acc_scale_factor);
    };

    void moveToFrameLinear(std::string frame_id, double z_offset_m = 0, double z_orientation_rad = 0)
    {
        tf2::Quaternion q;
        q.setRPY(0,0,z_orientation_rad);

        geometry_msgs::Pose offset;
        offset.position.x = 0;
        offset.position.y = 0;
        offset.position.z = -z_offset_m;
        offset.orientation.x = q.x();
        offset.orientation.y = q.y();
        offset.orientation.z = q.z();
        offset.orientation.w = q.w();
        moveToFrameLinear(frame_id, offset);
    }

    void moveToFramePTP(std::string frame_id, double z_offset_m = 0, double z_orientation_rad = 0)
    {
        tf2::Quaternion q;
        q.setRPY(0,0,z_orientation_rad);

        geometry_msgs::Pose offset;
        offset.position.x = 0;
        offset.position.y = 0;
        offset.position.z = -z_offset_m;
        offset.orientation.x = q.x();
        offset.orientation.y = q.y();
        offset.orientation.z = q.z();
        offset.orientation.w = q.w();
        moveToFramePTP(frame_id, offset);
    }

    void moveToHome()
    {
        // Home Position for Franka Emika
        moveToFramePTP("home");
    }

    void changeMaxVelocityScalingFactor(double max_vel_scale_factor)
    {
        mgi_->setMaxVelocityScalingFactor(max_vel_scale_factor);
    }

    void changeMaxAccelerationScalingFactor(double max_acc_scale_factor)
    {
        mgi_->setMaxAccelerationScalingFactor(max_acc_scale_factor);
    }

  private:

    void moveToFrameLinear(std::string frame_id, geometry_msgs::Pose & offset)
    {
        std::vector<geometry_msgs::Pose> waypoints;

        // Add Target Pose to waypoints
        waypoints.push_back(offset);


        #if DEBUG_VISUALIZATION
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration(1.0).sleep();

        geometry_msgs::PoseStamped offset_stamped;
        offset_stamped.header.frame_id = frame_id;
        offset_stamped.pose = offset;
        #endif
        // We want the Cartesian path to be interpolated at a resolution of 1 cm which is
        // why we will specify 0.01 as the max step in Cartesian translation.

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        mgi_->setPoseReferenceFrame(frame_id);
        mgi_->setEndEffectorLink("panda_hand_tcp");
        double fraction = mgi_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
        ROS_WARN_STREAM("Planning Frame: " << mgi_->getPlanningFrame());

        if (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            #if DEBUG_VISUALIZATION
            visual_tools_->publishAxisLabeled(tfBuffer.transform(offset_stamped, "panda_link0").pose, "target_point");
            // publish trajectory starting at frame panda_hand_tcp
            visual_tools_->publishPath(waypoints,rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
            visual_tools_->trigger();
            visual_tools_->prompt("execute trajectory");
            #endif
            mgi_->execute(trajectory);
        }
        else
        {
            ROS_ERROR("MoveItArmInterface::moveToFrameLinear: Failed to plan to target pose");
            throw std::runtime_error("MoveItArmInterface::moveToFrameLinear: Failed to plan to target pose");
        }
    }

    void moveToFramePTP(std::string frame_id, geometry_msgs::Pose & offset)
    {
        geometry_msgs::PoseStamped target_pose_stamped;
        target_pose_stamped.header.frame_id = frame_id;
        target_pose_stamped.pose = offset;

        mgi_->setPoseTarget(target_pose_stamped, "panda_hand_tcp");

        if (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            mgi_->move();
        else
        {
            ROS_ERROR("MoveItArmInterface::moveToFramePTP: Failed to plan to target pose");
            throw std::runtime_error("MoveItArmInterface::moveToFramePTP: Failed to plan to target pose");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
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

    void closeGripper()
    {
        mgi_->setJointValueTarget(mgi_->getNamedTargetValues("close"));

        if (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            mgi_->move();
    }

    void openGripper()
    {
        mgi_->setJointValueTarget(mgi_->getNamedTargetValues("open"));

        if (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            mgi_->move();
    }

    void setGripperWidth(double width)
    {
        std::vector<double> finger_width;
        finger_width.resize(2);
        finger_width[0] = width;
        finger_width[1] = width;

        mgi_->setJointValueTarget(finger_width);

        if (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            mgi_->move();
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_;
};

#endif