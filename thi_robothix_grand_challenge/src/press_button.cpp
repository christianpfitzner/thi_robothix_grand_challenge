#include "thi_robothix_grand_challenge/moveit_interface.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

int main(int argc, char** argv)
{
    //Sequence of Frames for the Robot to follow
    std::string poses[] = {
                            "box_button_blue",
                            "box_button_red",
                            "box_socket_red",
                            "box_socket_black",
                            "box_lid_handle_opened"
                            };

    ros::init(argc, argv, "move_group_interface_test");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create MoveItArmInterface
    MoveItArmInterface arm_interface(std::make_shared<moveit::planning_interface::MoveGroupInterface>("panda_arm"), 5.0, 1, 1);
    MoveItGripperInterface gripper_interface(std::make_shared<moveit::planning_interface::MoveGroupInterface>("panda_hand"), 5.0, 0.3, 0.1);

    // Move to home position
    arm_interface.moveToHome();

    // Close gripper
    gripper_interface.closeGripper();

    //Move to detection position
    arm_interface.moveToFrame("detection_pose");

    //Wait for detection & Localization 
    ros::Duration(5).sleep();

    //Move to Frames from poses[]
    for( std::string pose : poses)
    {
        arm_interface.moveToFrame(pose);
        ros::Duration(2).sleep();
    }

    ros::Duration(5).sleep();

    arm_interface.moveToHome();
    
    
    return 0;
}
