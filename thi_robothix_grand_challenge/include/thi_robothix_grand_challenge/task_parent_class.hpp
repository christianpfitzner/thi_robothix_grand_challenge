#ifndef TASK_CLASS_HPP
#define TASK_CLASS_HPP

#include "ros/ros.h"
#include <string>
#include "thi_robothix_grand_challenge/moveit_interface.hpp"
#include <memory>

class TaskClass
{
  public:
    TaskClass(std::string task_name);
    ~TaskClass() = default;
    virtual void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface) = 0;
    std::string _task_name = "Parent Task";
};

TaskClass::TaskClass(std::string task_name) : _task_name(task_name)
{
    ROS_INFO_STREAM("Initializing " << _task_name);
}

#endif