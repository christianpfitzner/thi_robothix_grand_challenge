#ifndef TASK_CLASS_HPP
#define TASK_CLASS_HPP

#include "ros/ros.h"
#include <string>
#include "thi_robothix_grand_challenge/moveit_interface.hpp"
#include <memory>

class TaskClass
{
  public:
    TaskClass();
    ~TaskClass() = default;
    std::string task_name;
    virtual void pre_run();
    virtual void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface) = 0;
};

TaskClass::TaskClass()
{
    ROS_INFO_STREAM("Initializing " << task_name);
}

void TaskClass::pre_run()
{
    ROS_INFO("\n\n--------------------");
    ROS_INFO_STREAM("Running " << task_name);
}

#endif