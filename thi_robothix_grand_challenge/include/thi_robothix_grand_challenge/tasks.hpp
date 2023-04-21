#ifndef TASK_IMPLEMENTATIONS_HPP
#define TASK_IMPLEMENTATIONS_HPP

#include "ros/ros.h"
#include "thi_robothix_grand_challenge/task_parent_class.hpp"
#include <string>

class Task_A : public TaskClass
{
  public:
    Task_A() : TaskClass("Task_A: Press Blue Button") {};
    using TaskClass::pre_run();
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);
};

class Task_B : public TaskClass
{
  public:
    Task_B() : TaskClass("Task_B: Adjust Slider") {};
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);
};

class Task_C : public TaskClass
{
  public:
    Task_C() : TaskClass("Task_C: Switch Sockets") {};
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);
};

class Task_D : public TaskClass
{
  public:
    Task_D() : TaskClass("Task_D: Open Flap and Probe") {};
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);
};

class Task_E : public TaskClass
{
  public:
    Task_E() : TaskClass("Task_E: Wrap Cable and insert Probe into white Socket") {};
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);
};

class Task_F : public TaskClass
{
  public:
    Task_F() : TaskClass("Task_F: Press Red Button") {};
    void run(std::unique_ptr<MoveItArmInterface> arm_interface, std::unique_ptr<MoveItGripperInterface> gripper_interface);
};


#endif