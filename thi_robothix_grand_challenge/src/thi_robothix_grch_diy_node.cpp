#include <ros/ros.h>
#include <ros/duration.h>
#include "thi_robothix_grand_challenge/moveit_interface.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <vector>
#include <memory>
#include <geometry_msgs/Point.h>
#include <string>
#include <unistd.h> 

int main(int argc, char **argv)
{

    ros::init(argc, argv, "thi_robothix_grand_challenge_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(10).sleep();

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("panda_link0");

    MoveItArmInterface arm_interface(std::make_unique<moveit::planning_interface::MoveGroupInterface>("panda_arm"), 0.5, 1, 1, visual_tools);
    MoveItGripperInterface gripper_interface(std::make_unique<moveit::planning_interface::MoveGroupInterface>("panda_hand"), 0.5, 0.3, 0.1);

    arm_interface.moveToHome();

    //go to start position
    arm_interface.moveToFrameLinear("bat_det_pose",0);

    /********************/
    /*Wait for detection*/
    /********************/


    //gripping pose for battery, to be defined
    arm_interface.moveToFrameLinear("gripping_pose",0);

    gripper_interface.setGripperWidthWithEffort(0.35);

    arm_interface.moveToFrameLinear("bat_det_pose", 0.1);
    arm_interface.moveToFramePTP("container",0.1);
    arm_interface.moveToFrameLinear("container",0);

    gripper_interface.openGripper();

    arm_interface.moveToFrameLinear("container",0.1);

    arm_interface.moveToFramePTP("disp_r_pose",0.1);
    arm_interface.moveToFrameLinear("disp_r_pose",0);

    /******************/
    /*Wait for reading*/
    /******************/


    arm_interface.moveToHome();
    ROS_INFO("DIY Task Finished!");

    return 0;
}
