#include "thi_robothix_grand_challenge/moveit_interface.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <tf/exceptions.h>

#include <memory>
#include <thread>

void trigger_box_detection(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<bool> box_detected)
{
    // service to trigger box detection
    ros::ServiceClient client=(*nh).serviceClient<std_srvs::Trigger>("box_detection");
    std_srvs::Trigger srv_call;
    ros::service::waitForService("box_detection", ros::Duration(5));
    if (!client.call(srv_call))
    {
        ROS_ERROR("Failed to call service box_detection");
        *box_detected = false;

    }
    else
    {
        if (srv_call.response.success)
        {
            ROS_INFO("box_detection service call success");
            *box_detected = true;
        }
        else
        {
            ROS_ERROR("box_detection service call failed");
            *box_detected = false;
        }
    }
}

void trigger_box_feature_detection(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<bool> box_feature_detected)
{
    // service to trigger box detection
    ros::ServiceClient client=(*nh).serviceClient<std_srvs::Trigger>("box_feature_detection");
    std_srvs::Trigger srv_call;
    ros::service::waitForService("box_feature_detection", ros::Duration(5));
    if (!client.call(srv_call))
    {
        ROS_ERROR("Failed to call service box_feature_detection");
        *box_feature_detected = false;

    }
    else
    {
        if (srv_call.response.success)
        {
            ROS_INFO("box_feature_detection service call success");
            *box_feature_detected = true;
        }
        else
        {
            ROS_ERROR("box_feature_detection service call failed");
            *box_feature_detected = false;
        }
    }
}

int main(int argc, char** argv)
{
    
/* 
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │ Initialize ROS                                                              │
    └─────────────────────────────────────────────────────────────────────────────┘
*/

    ros::init(argc, argv, "move_group_interface_test");
    auto nh = std::make_shared<ros::NodeHandle>();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create MoveItArmInterface
    MoveItArmInterface arm_interface(std::make_shared<moveit::planning_interface::MoveGroupInterface>("panda_arm"), 5.0, 1, 1);
    MoveItGripperInterface gripper_interface(std::make_shared<moveit::planning_interface::MoveGroupInterface>("panda_hand"), 5.0, 0.3, 0.1);

/* 
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │ Move to home position                                                       │
    └─────────────────────────────────────────────────────────────────────────────┘
*/
    arm_interface.moveToHome();


/* 
    ┌───────────────────────────────────────────────────────────────────────────-─┐
    │ parallerely detect box and close gripper                                    │
    └────────────────────────────────────────────────────────────────────────────-┘
*/
    auto box_detected = std::make_shared<bool>(false);
    std::thread trigger_box_detection_thread(trigger_box_detection, nh, box_detected);

    // Close gripper
    gripper_interface.closeGripper();

    // wait for box detection
    trigger_box_detection_thread.join();

    if (!(*box_detected))
    {
        ROS_ERROR("Box not detected");
        arm_interface.moveToHome();
        return 0;
    }
    

/* 
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │ after detecting the box move above the box (>detection_pose<) and           │
    │ trigger box feature detection                                               │
    └─────────────────────────────────────────────────────────────────────────────┘
*/
    auto box_feature_detected = std::make_shared<bool>(false);

    //Move to detection position
    arm_interface.moveToFramePTP("detection_pose");

    // trigger box feature detection
    std::thread trigger_box_feature_detection_thread(trigger_box_feature_detection, nh, box_feature_detected);

    // wait for img capture
    ros::Duration(1).sleep();

    // lower arm to box
    try
    {
        arm_interface.approachFrameLinear("detection_pose", -10);
    }
    catch (tf::LookupException & e)
    {
        ROS_ERROR("TF Lookup Exception: %s", e.what());
        arm_interface.moveToHome();
        return 0;
    }

    // wait for box feature detection
    trigger_box_feature_detection_thread.join();

    if(!(*box_feature_detected))
    {
        ROS_ERROR("Box features not detected");
        arm_interface.moveToHome();
        return 0;
    }


/*
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │ Validate position detection by pressing red button                          │
    └─────────────────────────────────────────────────────────────────────────────┘
*/
    arm_interface.approachFramePTP("box_button_red", 0.05);
    arm_interface.moveToFrameLinear("box_button_red");
    
    
    return 0;
}
