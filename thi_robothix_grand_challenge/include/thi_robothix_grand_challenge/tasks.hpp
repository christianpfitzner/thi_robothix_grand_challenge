#ifndef TASK_IMPLEMENTATIONS_HPP
#define TASK_IMPLEMENTATIONS_HPP

#include "ros/ros.h"
#include "thi_robothix_grand_challenge/task_parent_class.hpp"
#include <string>

class Task_A : public TaskClass
{
  public:
    using TaskClass::TaskClass;
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);

    std::string task_name = "Task_A: Slider";
};

class Task_B : public TaskClass
{
  public:
    using TaskClass::TaskClass;

    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);

    std::string task_name = "Task_B: TODO";

};

class Task_C : public TaskClass
{
  public:
    using TaskClass::TaskClass;
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);

    std::string task_name = "Task_C: TODO";

};

class Task_D : public TaskClass
{
  public:
    using TaskClass::TaskClass;
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);

    std::string task_name = "Task_D: TODO";
};

class Task_E : public TaskClass
{
  public:
    using TaskClass::TaskClass;
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);

    std::string task_name = "Task_E: TODO";
};


#endif