#include "ros/ros.h"
#include "thi_robothix_grand_challenge/task_parent_class.hpp"
#include "thi_robothix_grand_challenge/tasks.hpp"
#include "thi_robothix_grand_challenge/moveit_interface.hpp"
#include <memory>
#include <string>
#include "moveit/move_group_interface/move_group_interface.h"
#include <vector>
#include "std_srvs/Trigger.h"
#include <stdexcept>
#include <thread>
#include <tf/tf.h>

inline std::vector<std::unique_ptr<TaskClass>> create_tasks(std::string task_order)
{
    std::vector<std::unique_ptr<TaskClass>> tasks;
    tasks.reserve(6);

    if(task_order.size() != 6)
    {
        ROS_ERROR("Invalid task order.");
    }

    ROS_INFO_STREAM("Task order: " << task_order);

    // read task order and create tasks
    for(auto &&c : task_order)
    {
        switch(c)
        {
            case 'A':
                tasks.push_back(std::make_unique<Task_A>());
                break;
            case 'B':
                tasks.push_back(std::make_unique<Task_B>());
                break;
            case 'C':
                tasks.push_back(std::make_unique<Task_C>());
                break;
            case 'D':
                tasks.push_back(std::make_unique<Task_D>());
                break;
            case 'E':
                tasks.push_back(std::make_unique<Task_E>());
                break;
            case 'F':
                tasks.push_back(std::make_unique<Task_F>());
                break;
            default:
                ROS_ERROR_STREAM("Invalid task order");
                break;
        }
    }
    return tasks;
}

void trigger_box_detection(std::shared_ptr<ros::NodeHandle> nh)
{
    // service to trigger box detection
    ros::ServiceClient client=(*nh).serviceClient<std_srvs::Trigger>("box_detection");
    std_srvs::Trigger srv_call;
    ros::service::waitForService("box_detection", ros::Duration(5));
    if (!client.call(srv_call))
    {
        ROS_ERROR("Failed to call service box_detection");
        throw std::runtime_error("Failed to call service box_detection");

    }
    else
    {
        if (srv_call.response.success)
        {
            ROS_INFO("box_detection service call success");
        }
        else
        {
            ROS_ERROR("box_detection service call failed");
            throw std::runtime_error("box_detection service call failed");
        }
    }
}

inline void detect_box(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    std::thread trigger_box_detection_thread(trigger_box_detection, nh);

    // Close gripper
    gripper_interface.closeGripper();

    // wait for box detection
    trigger_box_detection_thread.join();
}

void Task_A::run(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    gripper_interface.closeGripper();
    arm_interface.moveToFramePTP("box_button_blue",0.1,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_button_blue",0.006,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_button_blue",0.1,0,EE_LINKS::PANDA_HAND_BOTTOM);
    gripper_interface.setGripperWidth(0.035);
}

void Task_B::run(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    arm_interface.moveToFramePTP("box_slider_start",0.1,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    
    arm_interface.changeMaxVelocityScalingFactor(0.01);
    arm_interface.moveToFrameLinear("box_slider_start",0.005,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    gripper_interface.setGripperWidth_ABS(0.01);

    arm_interface.moveToFramePTP("box_slider_1",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFramePTP("box_slider_middle",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFramePTP("box_slider_2",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFramePTP("box_slider_stop",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);

    arm_interface.moveToFramePTP("box_slider_2",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFramePTP("box_slider_middle",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFramePTP("box_slider_1",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFramePTP("box_slider_start",0.008,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);

    //arm_interface.changeMaxVelocityScalingFactor(0.01);
    arm_interface.moveToFrameLinear("box_slider_start",0.1,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM); 
    gripper_interface.setGripperWidth(0.035);
    arm_interface.changeMaxVelocityScalingFactor(0.7);
    //gripper_interface.openGripper();   
}

void Task_C::run(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    arm_interface.moveToFramePTP("box_socket_black", 0.1,0,EE_LINKS::PANDA_HAND_BOTTOM);
    //gripper_interface.setGripperWidth(0.03);
    // wait 1 sec
    ros::Duration(1).sleep();
    arm_interface.moveToFrameLinear("box_socket_black",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.changeMaxVelocityScalingFactor(0.4);
    gripper_interface.setGripperWidth(0.01);
    ros::Duration(1).sleep();

    arm_interface.moveToFrameLinear("box_socket_black", 0.05,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_socket_red", 0.05,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_socket_red",0.04,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_socket_red",0.035,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_socket_red",0.03,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_socket_red",0.02,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_socket_red",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    // gripper_interface.setGripperWidth(0.035);
    // //arm_interface.changeMaxVelocityScalingFactor(0.01);
    // gripper_interface.setGripperWidth(0.0055);
    gripper_interface.openGripper();
    ros::Duration(1).sleep();
    arm_interface.moveToFrameLinear("box_socket_red", 0.1,0,EE_LINKS::PANDA_HAND_BOTTOM);
}

void Task_D::run(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    //grip the probe and change tcp
    //gripper_interface.setGripperWidth(0.05);
    //arm_interface.moveToFramePTP("box_probe_gripping_point", 0.05);
    //arm_interface.moveToFrameLinear("box_probe_gripping_point");
    //gripper_interface.closeGripper();
    //tf tcp to probe

    //arm_interface.moveToFrameLinear("box_socket_grey", 0.02);

    //open lid
    //arm_interface.moveToFrameLinear("box_lid_opening_pos_1", 0.05);
    //arm_interface.moveToFrameLinear("box_lid_opening_pos_1");
    //arm_interface.moveToFrameLinear("box_lid_opening_pos_2");
    //arm_interface.moveToFrameLinear("box_lid_opening_pos_3");

    //make a measurment
    //arm_interface.moveToFramePTP("box_measuring_point_1",0.05);
    //arm_interface.moveToFrameLinear("box_measuring_point_1");
    //arm_interface.moveToFrameLinear("box_measuring_point_1",0.05);
}

void Task_E::run(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    arm_interface.moveToFramePTP("box_cable_wrapping_pos_1",0.1,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_1",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_2",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_3",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);

    gripper_interface.closeGripper();

    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_4",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_5",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_6",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_7",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_cable_wrapping_pos_8",0.01,0,EE_LINKS::PANDA_HAND_BOTTOM);

    gripper_interface.openGripper();
}

void Task_F::run(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    gripper_interface.closeGripper();
    arm_interface.moveToFramePTP("box_button_red",0.1,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_button_red",0.006,0,EE_LINKS::PANDA_HAND_BOTTOM);
    arm_interface.moveToFrameLinear("box_button_red",0.1,0,EE_LINKS::PANDA_HAND_BOTTOM);
    gripper_interface.openGripper();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "thi_robothix_grand_challenge_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("panda_link0");

    MoveItArmInterface arm_interface(std::make_unique<moveit::planning_interface::MoveGroupInterface>("panda_arm"), 0.5, 0.5, 0.5, visual_tools);
    MoveItGripperInterface gripper_interface(std::make_unique<moveit::planning_interface::MoveGroupInterface>("panda_hand"), 0.5, 0.3, 0.1);

/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Create Tasks                                                                │
  └─────────────────────────────────────────────────────────────────────────────┘
*/
    std::string task_order = "ABCDEF";
    if(!nh.getParam("/task_order", task_order))
        ROS_ERROR("Could not get task order from parameter server. Using default task order.");

    std::vector<std::unique_ptr<TaskClass>> tasks = create_tasks(task_order);

/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ wait until ready                                                            │
  └─────────────────────────────────────────────────────────────────────────────┘
*/
    // ROS_WARN("Waiting for user to press enter to start.");
    // visual_tools->prompt("Press 'continue' in the RvizVisualToolsGui window to start execution.");
    
/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Detect Box & Get to Start Position                                          │
  └─────────────────────────────────────────────────────────────────────────────┘
 */
    arm_interface.moveToHome();
    // try
    // {
    //     detect_box(std::move(arm_interface), std::move(gripper_interface), std::make_shared<ros::NodeHandle>(nh));
    // }
    // catch(const std::runtime_error &e)
    // {
    //     ROS_ERROR_STREAM("Could not detect box. " << e.what());
    //     arm_interface->moveToHome();
    //     return 1;
    // }

/*
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Run Tasks                                                                   │
  └─────────────────────────────────────────────────────────────────────────────┘
*/
    while(1)
    {
        for(auto &&task : tasks)
        {
            ROS_WARN_STREAM("Trying to run task " << task->_task_name);
            task->run(arm_interface, gripper_interface, std::make_shared<ros::NodeHandle>(nh));
        }
    }

/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Move To Home Position                                                       │
  └─────────────────────────────────────────────────────────────────────────────┘
 */
    arm_interface.moveToHome();
    ROS_WARN("Finished all tasks.");

    return 0;
}
