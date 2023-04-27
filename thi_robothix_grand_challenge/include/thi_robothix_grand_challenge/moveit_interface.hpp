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
#include "geometry_msgs/TransformStamped.h"
#include "moveit_msgs/MotionPlanRequest.h"
#include "moveit_msgs/MotionPlanResponse.h"

#define DEBUG_VISUALIZATION false

enum class EE_LINKS {PANDA_HAND_TCP, PANDA_HAND_BOTTOM, PANDA_PROBE};


class MoveItArmInterface
{
  public:
    MoveItArmInterface(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi, const double planning_time, const double max_vel_scale_factor, const double max_acc_scale_factor, std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools) : mgi_(mgi), visual_tools_(visual_tools), tfListener_(tfBuffer_)
    {
        mgi_->setPlanningTime(planning_time);
        mgi_->setMaxVelocityScalingFactor(max_vel_scale_factor);
        mgi_->setMaxAccelerationScalingFactor(max_acc_scale_factor);
        
        ros::Duration(1.0).sleep();

        // get tf transform from panda_link0 to box_base_link
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer_.lookupTransform("panda_link0", "box_center_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // collision check
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = mgi_->getPlanningFrame();
        collision_object.id = "box";
        shape_msgs::SolidPrimitive box_primitive;
        box_primitive.type = box_primitive.BOX;
        box_primitive.dimensions.resize(3);
        box_primitive.dimensions[box_primitive.BOX_X] = 0.156;
        box_primitive.dimensions[box_primitive.BOX_Y] = 0.257;
        box_primitive.dimensions[box_primitive.BOX_Z] = 0.0794;

        geometry_msgs::Pose box_pose;
        box_pose.orientation =  transformStamped.transform.rotation;
        box_pose.position.x = transformStamped.transform.translation.x;
        box_pose.position.y = transformStamped.transform.translation.y;
        box_pose.position.z = transformStamped.transform.translation.z;

        shape_msgs::SolidPrimitive floor_primitive;
        floor_primitive.type = floor_primitive.BOX;
        floor_primitive.dimensions.resize(3);
        floor_primitive.dimensions[floor_primitive.BOX_X] = 1;
        floor_primitive.dimensions[floor_primitive.BOX_Y] = 1;
        floor_primitive.dimensions[floor_primitive.BOX_Z] = 0.05;

        geometry_msgs::Pose floor_pose;
        floor_pose.orientation.w = 1.0;
        floor_pose.position.x = 0;
        floor_pose.position.y = 0;
        floor_pose.position.z = 0.025;
        
        shape_msgs::SolidPrimitive lid_primitive;
        lid_primitive.type = lid_primitive.BOX;
        lid_primitive.dimensions.resize(3);
        lid_primitive.dimensions[lid_primitive.BOX_X] = 0.08;
        lid_primitive.dimensions[lid_primitive.BOX_Y] = 0.06;
        lid_primitive.dimensions[lid_primitive.BOX_Z] = 0.08;

        geometry_msgs::PoseStamped lid_pose;
        lid_pose.header.frame_id = "box_lid_opening_pos_2";
        lid_pose.pose.orientation.w = 1;
        lid_pose.pose.position.x = -0.04;
        lid_pose.pose.position.y = -0.03;
        lid_pose.pose.position.z = 0.04;

        lid_pose = tfBuffer_.transform(lid_pose, "panda_link0");

        collision_object.primitives.push_back(box_primitive);
        collision_object.primitives.push_back(floor_primitive);
        collision_object.primitives.push_back(lid_primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.primitive_poses.push_back(floor_pose);
        collision_object.primitive_poses.push_back(lid_pose.pose);
        collision_object.operation = collision_object.ADD;

        collision_objects_.push_back(collision_object);
        planning_scene_interface_.addCollisionObjects(collision_objects_);

        ros::Duration(1.0).sleep();
    };

    
    void do_trajectory(std::vector<geometry_msgs::Pose> waypoints)
    {
        ROS_WARN("do trajectory0");;
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        //mgi_->setPoseReferenceFrame(frame_id);
        mgi_->setEndEffectorLink("panda_hand_tcp");
        double fraction = mgi_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    }

    void moveToFrameLinear(std::string frame_id, double z_offset_m = 0, double z_orientation_rad = 0, EE_LINKS ee_link=EE_LINKS::PANDA_HAND_TCP)
    {
        geometry_msgs::Pose offset;

        geometry_msgs::TransformStamped transform_link0_tcp;
        transform_link0_tcp = tfBuffer_.lookupTransform("panda_link0", "panda_hand_tcp", ros::Time(0));

        if(ee_link == EE_LINKS::PANDA_HAND_TCP)
        {
            // set position
            offset.position.x = 0;
            offset.position.y = 0;
            offset.position.z = 0 + z_offset_m;

            // set orientation
            tf2::Quaternion q;
            q.setRPY(0,0,z_orientation_rad);

            offset.orientation.x = q.x();
            offset.orientation.y = q.y();
            offset.orientation.z = q.z();
        }
        else if(ee_link == EE_LINKS::PANDA_HAND_BOTTOM)
        {
            // lookup transforms
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer_.lookupTransform("panda_hand_tcp", "panda_hand_bottom", ros::Time(0), ros::Duration(1.0));

            // set position
            offset.position.x = transformStamped.transform.translation.x;
            offset.position.y = transformStamped.transform.translation.y;
            offset.position.z = transformStamped.transform.translation.z + z_offset_m;

            // set orientation
            tf2::Quaternion q;
            q.setRPY(0,0,z_orientation_rad);
            tf2::Quaternion q_tf;
            q_tf.setX(transformStamped.transform.rotation.x);
            q_tf.setY(transformStamped.transform.rotation.y);
            q_tf.setZ(transformStamped.transform.rotation.z);
            q_tf.setW(transformStamped.transform.rotation.w);
            q = q_tf * q * q_tf.inverse();
            offset.orientation.x = q.x();
            offset.orientation.y = q.y();
            offset.orientation.z = q.z();
        }
        else if(ee_link == EE_LINKS::PANDA_PROBE)
        {
            // lookup transforms
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer_.lookupTransform("panda_hand_tcp", "panda_hand_probe", ros::Time(0), ros::Duration(1.0));

            // set position
            offset.position.x = transformStamped.transform.translation.x;
            offset.position.y = transformStamped.transform.translation.y;
            offset.position.z = transformStamped.transform.translation.z + z_offset_m;

            // set orientation
            tf2::Quaternion q;
            q.setRPY(0,0,z_orientation_rad);
            tf2::Quaternion q_tf;
            q_tf.setX(transformStamped.transform.rotation.x);
            q_tf.setY(transformStamped.transform.rotation.y);
            q_tf.setZ(transformStamped.transform.rotation.z);
            q_tf.setW(transformStamped.transform.rotation.w);
            q = q_tf * q * q_tf.inverse();
            offset.orientation.x = q.x();
            offset.orientation.y = q.y();
            offset.orientation.z = q.z();
        }

        else
            throw std::invalid_argument("Invalid EE_LINKS");

        offset.position.x += transform_link0_tcp.transform.translation.x;
        offset.position.y += transform_link0_tcp.transform.translation.y;
        offset.position.z += transform_link0_tcp.transform.translation.z;
        tf2::Quaternion q;
        q.setX(offset.orientation.x);
        q.setY(offset.orientation.y);
        q.setZ(offset.orientation.z);
        q.setW(offset.orientation.w);
        tf2::Quaternion q_tf;
        q_tf.setX(transform_link0_tcp.transform.rotation.x);
        q_tf.setY(transform_link0_tcp.transform.rotation.y);
        q_tf.setZ(transform_link0_tcp.transform.rotation.z);
        q_tf.setW(transform_link0_tcp.transform.rotation.w);
        q = q_tf * q * q_tf.inverse();
        offset.orientation.x = q.x();
        offset.orientation.y = q.y();
        offset.orientation.z = q.z();
        offset.orientation.w = q.w();

        moveToFrameLinear(frame_id, offset);
    }

    void moveToFramePTP(std::string frame_id, double z_offset_m = 0, double z_orientation_rad = 0, EE_LINKS ee_link=EE_LINKS::PANDA_HAND_TCP)
    {

        geometry_msgs::PoseStamped offset;

        tf2::Quaternion q;
        q.setRPY(0,0,z_orientation_rad);

        offset.pose.position.x = 0;
        offset.pose.position.y = 0;
        offset.pose.position.z = -z_offset_m;
        offset.pose.orientation.x = q.x();
        offset.pose.orientation.y = q.y();
        offset.pose.orientation.z = q.z();
        offset.pose.orientation.w = q.w();

        switch (ee_link)
        {
        case EE_LINKS::PANDA_HAND_TCP:
            offset.header.frame_id = "panda_hand_tcp";
            break;
        case EE_LINKS::PANDA_HAND_BOTTOM:
            offset.header.frame_id = "panda_hand_tcp";
            offset = tfBuffer_.transform(offset, "panda_hand_bottom");

            break;
        case EE_LINKS::PANDA_PROBE:
            offset.header.frame_id = "panda_hand_tcp";
            offset = tfBuffer_.transform(offset, "panda_probe");
            break;
        
        default:
            throw std::invalid_argument("Invalid EE_LINKS");
            break;
        }

        moveToFramePTP(frame_id, offset.pose);
    }

    void moveToHome()
    {
        // Home Position for Franka Emika
        moveToFramePTP("home",0,0,EE_LINKS::PANDA_HAND_TCP);
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

    // void moveToFrameLinearVel(std::string frame_id, geometry_msgs::Pose & offset, double max_vel, double max_acc)
    // {
    //     moveit_msgs::MotionPlanRequest req;
    //     req.group_name = "panda_arm";
    //     req.max_velocity_scaling_factor = max_vel;
    //     req.max_acceleration_scaling_factor = max_acc;
    //     req.allowed_planning_time = mgi_->getPlanningTime();
    //     req.start_state.joint_state = mgi_->getRobotModel()->("panda_arm");
    // }

    void moveToFrameLinear(std::string frame_id, geometry_msgs::Pose & offset)
    {
        // reset planning scene
        planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1,"box"));

        // change planner to pilz motion planner linear
        mgi_->setPlanningPipelineId("pilz_industrial_motion_planner");
        mgi_->setPlannerId("LIN");

        std::vector<geometry_msgs::Pose> waypoints;

        // Add Target Pose to waypoints
        waypoints.push_back(offset);

        // We want the Cartesian path to be interpolated at a resolution of 1 cm which is
        // why we will specify 0.01 as the max step in Cartesian translation.
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        // reset plan
        my_plan_arm_ = moveit::planning_interface::MoveGroupInterface::Plan();

        mgi_->setPoseReferenceFrame("panda_link0");
        mgi_->setEndEffectorLink("panda_hand_tcp");
        mgi_->setPoseTarget(offset);
        mgi_->setNumPlanningAttempts(20);
        mgi_->setPlanningTime(5.0);
        mgi_->setReplanAttempts(20);
        moveit::planning_interface::MoveItErrorCode error_code = mgi_->plan(my_plan_arm_);
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS) 
        {
            // visualization
            visual_tools_->deleteAllMarkers();
            visual_tools_->setBaseFrame("panda_link0");
            visual_tools_->publishAxisLabeled(offset, "LIN Goal");
            visual_tools_->publishTrajectoryLine(my_plan_arm_.trajectory_, mgi_->getCurrentState()->getJointModelGroup("panda_arm"));
            visual_tools_->trigger();
            visual_tools_->prompt("Press 'next' to move to target pose");

            mgi_->move();
        }

        else if(error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN)
        {
            throw std::runtime_error("INVALID_MOTION_PLAN");
        }
        else if(error_code == moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION)
        {
            throw std::runtime_error("GOAL_IN_COLLISION");
        }
    }

    void moveToFramePTP(std::string frame_id, geometry_msgs::Pose & offset)
    {
        // set planning scene
        planning_scene_interface_.addCollisionObjects(collision_objects_);


        geometry_msgs::PoseStamped target_pose_stamped;
        target_pose_stamped.header.frame_id = frame_id;
        target_pose_stamped.pose = offset;


        my_plan_arm_ = moveit::planning_interface::MoveGroupInterface::Plan();

        // change planner to default planner        
        mgi_->setPlanningPipelineId("ompl");
        mgi_->setPlannerId("geometric::RRTConnect");
        mgi_->setPoseTarget(target_pose_stamped, "panda_hand_tcp");

        if (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            visual_tools_->deleteAllMarkers();
            visual_tools_->setBaseFrame(frame_id);
            visual_tools_->publishAxisLabeled(offset, "PTP Goal", rviz_visual_tools::MEDIUM, rviz_visual_tools::MAGENTA);
            visual_tools_->publishTrajectoryLine(my_plan_arm_.trajectory_, mgi_->getCurrentState()->getJointModelGroup("panda_arm"));
            visual_tools_->trigger();
            visual_tools_->prompt("Press 'next' to move to target pose");

            mgi_->move();
        }
        else
        {
            ROS_ERROR("MoveItArmInterface::moveToFramePTP: Failed to plan to target pose");
            throw std::runtime_error("MoveItArmInterface::moveToFramePTP: Failed to plan to target pose");
        }
    }

    geometry_msgs::TransformStamped hand_to_hand_bottom_;
    geometry_msgs::TransformStamped hand_to_probe_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::vector<moveit_msgs::CollisionObject> collision_objects_;
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