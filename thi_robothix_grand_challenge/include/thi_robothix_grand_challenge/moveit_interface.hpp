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


class MoveItArmInterface
{
  public:
    MoveItArmInterface(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi, const double planning_time, const double max_vel_scale_factor, const double max_acc_scale_factor) : mgi_(mgi) {
        mgi_->setPlanningTime(planning_time);
        mgi_->setMaxVelocityScalingFactor(max_vel_scale_factor);
        mgi_->setMaxAccelerationScalingFactor(max_acc_scale_factor);
    };

    bool moveToFrame(std::string frame_id)
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
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();

            return success; 
        }

        geometry_msgs::PoseStamped target_pose; 
        target_pose.header.frame_id    = "panda_link0";

        try
        {
            geometry_msgs::TransformStamped transform_hand_to_tcp = tfBuffer.lookupTransform("panda_hand", "panda_hand_tcp_inv", ros::Time(0));

            target_pose.pose.position.x    = transformStamped.transform.translation.x + transform_hand_to_tcp.transform.translation.x; 
            target_pose.pose.position.y    = transformStamped.transform.translation.y + transform_hand_to_tcp.transform.translation.y; 
            target_pose.pose.position.z    = transformStamped.transform.translation.z + transform_hand_to_tcp.transform.translation.z;

            target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.pose.orientation.x = transformStamped.transform.rotation.x;    
            target_pose.pose.orientation.y = transformStamped.transform.rotation.y;    
            target_pose.pose.orientation.z = transformStamped.transform.rotation.z;// + 0.78539816339744830961566084581988;   
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();

            target_pose.pose.position.x    = transformStamped.transform.translation.x; 
            target_pose.pose.position.y    = transformStamped.transform.translation.y; 
            target_pose.pose.position.z    = transformStamped.transform.translation.z; 

            target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.pose.orientation.x = transformStamped.transform.rotation.x;    
            target_pose.pose.orientation.y = transformStamped.transform.rotation.y;    
            target_pose.pose.orientation.z = transformStamped.transform.rotation.z;   
        }

        mgi_->setPoseTarget(target_pose);

        success = (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
            mgi_->move();
        return success; 
    }

    bool approachFrame(std::string frame_id, double z_offset_mm)
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
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();

            return success; 
        }


        geometry_msgs::PoseStamped target_pose; 
        target_pose.header.frame_id    = "panda_link0";

        try
        {
            geometry_msgs::TransformStamped transform_hand_to_tcp = tfBuffer.lookupTransform("panda_hand", "panda_hand_tcp", ros::Time(0));

            target_pose.pose.position.x    = transformStamped.transform.translation.x + transform_hand_to_tcp.transform.translation.x; 
            target_pose.pose.position.y    = transformStamped.transform.translation.y + transform_hand_to_tcp.transform.translation.y; 
            target_pose.pose.position.z    = transformStamped.transform.translation.z + transform_hand_to_tcp.transform.translation.z + z_offset_mm;

            target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.pose.orientation.x = transformStamped.transform.rotation.x;    
            target_pose.pose.orientation.y = transformStamped.transform.rotation.y;    
            target_pose.pose.orientation.z = transformStamped.transform.rotation.z;// + 1.570796;   
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();

            target_pose.pose.position.x    = transformStamped.transform.translation.x; 
            target_pose.pose.position.y    = transformStamped.transform.translation.y; 
            target_pose.pose.position.z    = transformStamped.transform.translation.z; 

            target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
            target_pose.pose.orientation.x = transformStamped.transform.rotation.x;    
            target_pose.pose.orientation.y = transformStamped.transform.rotation.y;    
            target_pose.pose.orientation.z = transformStamped.transform.rotation.z;   
        }   

        mgi_->setPoseTarget(target_pose);

        success = (mgi_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success)
            mgi_->move();

        return success; 
    }

    bool moveToHome()
    {
        //Home Position for Franka NOT Defined! Needs to be defined. 
        return moveToFrame("home"); 
    }

  private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_;
};

class MoveItGripperInterface
{
  public:
    MoveItGripperInterface(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi, const double planning_time, const double max_vel_scale_factor, const double max_acc_scale_factor) : mgi_(mgi) {
        mgi_->setPlanningTime(planning_time);
        mgi_->setMaxVelocityScalingFactor(max_vel_scale_factor);
        mgi_->setMaxAccelerationScalingFactor(max_acc_scale_factor);
    };

    bool closeGripper()
    {
        mgi_->setJointValueTarget(mgi_->getNamedTargetValues("close"));

        bool success = (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success) mgi_->move();

        return success; 
    }
    
    bool openGripper()
    {
        mgi_->setJointValueTarget(mgi_->getNamedTargetValues("open"));

        bool success = (mgi_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success) mgi_->move();

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
    
        if(success)
            mgi_->move();


        return success; 
    }

  private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_;
};

# endif