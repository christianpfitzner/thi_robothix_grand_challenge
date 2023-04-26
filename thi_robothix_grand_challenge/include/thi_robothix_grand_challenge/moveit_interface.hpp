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

    void approachFrameLinear(std::string frame_id, double z_offset_m)
    {
        auto offset = geometry_msgs::Pose();
        offset.position.x = 0;
        offset.position.y = 0;
        offset.position.z = z_offset_m;
        offset.orientation.x = 0;
        offset.orientation.y = 0;
        offset.orientation.z = 0;
        offset.orientation.w = 0;
        moveToFrameLinear(frame_id, offset);
    }

    void approachFramePTP(std::string frame_id, double z_offset_m)
    {
        auto offset = geometry_msgs::Pose();
        offset.position.x = 0;
        offset.position.y = 0;
        offset.position.z = z_offset_m;
        offset.orientation.x = 0;
        offset.orientation.y = 0;
        offset.orientation.z = 0;
        offset.orientation.w = 0;
        moveToFramePTP(frame_id, offset);
    }

    void moveToFrameLinear(std::string frame_id)
    {
        auto offset = geometry_msgs::Pose();
        offset.position.x = 0;
        offset.position.y = 0;
        offset.position.z = 0;
        offset.orientation.x = 0;
        offset.orientation.y = 0;
        offset.orientation.z = 0;
        offset.orientation.w = 0;
        moveToFrameLinear(frame_id, offset);
    }

    void moveToFramePTP(std::string frame_id)
    {
        auto offset = geometry_msgs::Pose();
        offset.position.x = 0;
        offset.position.y = 0;
        offset.position.z = 0;
        offset.orientation.x = 0;
        offset.orientation.y = 0;
        offset.orientation.z = 0;
        offset.orientation.w = 0;
        moveToFramePTP(frame_id, offset);
    }

    void moveToHome()
    {
        // Home Position for Franka Emika
        moveToFramePTP("home");
    }

  private:

    void moveToFrameLinear(std::string frame_id, geometry_msgs::Pose & offset)
    {
        std::vector<geometry_msgs::Pose> waypoints;

        geometry_msgs::Pose target_pose;
        geometry_msgs::TransformStamped transformStamped;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Duration(1.0).sleep();


        // must be transformed from world and not
        // from link0, because Pose is beign used here and not PoseStamped
        transformStamped = tfBuffer.lookupTransform("world", frame_id, ros::Time(0));

        geometry_msgs::TransformStamped transform_hand_to_tcp = tfBuffer.lookupTransform("panda_hand", "panda_hand_tcp", ros::Time(0));
        target_pose.position.x = transformStamped.transform.translation.x + transform_hand_to_tcp.transform.translation.x + offset.position.x;
        target_pose.position.y = transformStamped.transform.translation.y + transform_hand_to_tcp.transform.translation.y + offset.position.y;
        target_pose.position.z = transformStamped.transform.translation.z + transform_hand_to_tcp.transform.translation.z + offset.position.z;
        target_pose.orientation.w = transformStamped.transform.rotation.w; // * offset.orientation.w;
        target_pose.orientation.x = transformStamped.transform.rotation.x; // * offset.orientation.x;
        target_pose.orientation.y = transformStamped.transform.rotation.y; // * offset.orientation.y;
        target_pose.orientation.z = transformStamped.transform.rotation.z; // * offset.orientation.z;

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

        if (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            mgi_->execute(trajectory);
    }

    void moveToFramePTP(std::string frame_id, geometry_msgs::Pose & offset)
    {
        geometry_msgs::TransformStamped transformStamped;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Duration(1.0).sleep();

        transformStamped = tfBuffer.lookupTransform("panda_link0", frame_id, ros::Time(0));

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "panda_link0";

        geometry_msgs::TransformStamped transform_hand_to_tcp = tfBuffer.lookupTransform("panda_hand", "panda_hand_tcp", ros::Time(0));

        target_pose.pose.position.x = transformStamped.transform.translation.x + transform_hand_to_tcp.transform.translation.x + offset.position.x;
        target_pose.pose.position.y = transformStamped.transform.translation.y + transform_hand_to_tcp.transform.translation.y + offset.position.y;
        target_pose.pose.position.z = transformStamped.transform.translation.z + transform_hand_to_tcp.transform.translation.z + offset.position.z;
        target_pose.pose.orientation.w = transformStamped.transform.rotation.w; // * offset.orientation.w;
        target_pose.pose.orientation.x = transformStamped.transform.rotation.x; // * offset.orientation.x;
        target_pose.pose.orientation.y = transformStamped.transform.rotation.y; // * offset.orientation.y;
        target_pose.pose.orientation.z = transformStamped.transform.rotation.z; // * offset.orientation.z;


        mgi_->setPoseTarget(target_pose);

        if (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            mgi_->move();
    }

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