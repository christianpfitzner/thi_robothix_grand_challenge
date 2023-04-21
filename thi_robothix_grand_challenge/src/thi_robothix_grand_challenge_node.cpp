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

inline std::vector<std::shared_ptr<TaskClass>> create_tasks(std::string task_order)
{
    std::vector<std::shared_ptr<TaskClass>> tasks;
    tasks.resize(5);

    if(task_order.size() != 5)
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
                tasks.push_back(std::make_shared<Task_A>());
                break;
            case 'B':
                tasks.push_back(std::make_shared<Task_B>());
                break;
            case 'C':
                tasks.push_back(std::make_shared<Task_C>());
                break;
            case 'D':
                tasks.push_back(std::make_shared<Task_D>());
                break;
            case 'E':
                tasks.push_back(std::make_shared<Task_E>());
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

inline void detect_box(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface, std::shared_ptr<ros::NodeHandle> nh)
{
    auto box_detected = std::make_shared<bool>(false);
    std::thread trigger_box_detection_thread(trigger_box_detection, nh);

    // Close gripper
    gripper_interface->closeGripper();

    // wait for box detection
    trigger_box_detection_thread.join();
}

void Task_A::run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface)
{
    this->pre_run();
    //arm_inteface.
}

void Task_B::run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface)
{
    
}

void Task_C::run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface)
{
    
}

void Task_D::run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface)
{
    
}

void Task_E::run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface)
{
    
}

void Task_F::run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thi_robothix_grand_challenge_node");
    ros::NodeHandle nh;

    std::unique_ptr<MoveItArmInterface> arm_interface = std::make_unique<MoveItArmInterface>(std::make_unique<moveit::planning_interface::MoveGroupInterface>("panda_arm"), 0.5, 1, 1);
    std::unique_ptr<MoveItGripperInterface> gripper_interface = std::make_unique<MoveItGripperInterface>(std::make_unique<moveit::planning_interface::MoveGroupInterface>("panda_hand"), 0.5, 0.3, 0.1);

/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Create Tasks                                                                │
  └─────────────────────────────────────────────────────────────────────────────┘
*/
    std::string task_order = "ABCDE";
    if(!nh.getParam("/task_order", task_order))
        ROS_ERROR("Could not get task order from parameter server. Using default task order.");

    std::vector<std::shared_ptr<TaskClass>> tasks = create_tasks(task_order);

/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ wait until ready                                                            │
  └─────────────────────────────────────────────────────────────────────────────┘
*/
    //! TODO


/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Detect Box & Get to Start Position                                          │
  └─────────────────────────────────────────────────────────────────────────────┘
 */
    arm_interface->moveToHome();
    try
    {
        detect_box(std::move(arm_interface), std::move(gripper_interface), std::make_shared<ros::NodeHandle>(nh));
    }
    catch(const std::runtime_error &e)
    {
        ROS_ERROR_STREAM("Could not detect box. " << e.what());
        arm_interface->moveToHome();
        return 1;
    }

/*
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Run Tasks                                                                   │
  └─────────────────────────────────────────────────────────────────────────────┘
*/
    for(auto &&task : tasks)
    {
        task->run(std::move(arm_interface), std::move(gripper_interface));
    }

/* 
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ Move To Home Position                                                       │
  └─────────────────────────────────────────────────────────────────────────────┘
 */
    arm_interface->moveToHome();
    ROS_INFO("Finished all tasks.");

    return 0;
}
