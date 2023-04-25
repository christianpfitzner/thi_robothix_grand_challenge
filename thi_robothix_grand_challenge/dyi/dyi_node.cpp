#include <ros/ros.h>
#include <ros/duration.h>
#include "thi_robothix_grand_challenge/moveit_interface.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <vector>
#include <memory>
#include <geometry_msgs/Point.h>
#include <string>
#include <unistd.h> 


std::vector<geometry_msgs::Pose> wannabe_g_code_fn(std::string frame_id,  std::vector<geometry_msgs::Transform>& tf_list)
{

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::PoseStamped start_pose_stamped;
    start_pose_stamped.header.frame_id = frame_id;

    geometry_msgs::Pose target_pose = start_pose_stamped.pose;

    waypoints.push_back(target_pose);

    for(auto &&tf : tf_list)
    {
        
        target_pose.position.x += tf.translation.x;
        target_pose.position.y += tf.translation.y;
        target_pose.position.z += tf.translation.z;

        waypoints.push_back(target_pose);

        ROS_WARN_STREAM("target_pose:" << target_pose.position.x << " " <<
                                          target_pose.position.y
                                << " " << target_pose.position.z);
    }


    return waypoints;
}

void txt_from_robot(MoveItArmInterface& arm_interface, MoveItGripperInterface& gripper_interface,std::string fname)
{

    std::string cwd("\0",FILENAME_MAX+1);
    ROS_WARN_STREAM("CWD: "<< getcwd(&cwd[0],cwd.capacity()));

    //read file and convert to x,y,z translation
    std::ifstream csvFile;
    std::string strPathCSVFile = fname;
    csvFile.open(strPathCSVFile.c_str());   

    if (!csvFile.is_open())
    {
        ROS_ERROR("Wrong Path!");
        exit(EXIT_FAILURE);
    }

    std::vector<double> delta_x;
    std::vector<double> delta_y;
    std::vector<double> delta_z;

    std::string line;
    std::vector<std::string> vec;

    while (getline(csvFile,line))
    {
        if (line.empty()) // skip empty lines:
        {
            //cout << "empty line!" << endl;
            continue;
        }

        std::istringstream iss(line);
        std::string lineStream;
        std::string::size_type sz;

        std::vector<double> row;

        while (getline(iss, lineStream, ','))
        {  
            row.push_back(stold(lineStream,&sz)); // convert to double
        }

        delta_x.push_back(row[0]);
        delta_y.push_back(row[1]);
        delta_z.push_back(row[2]);

        ROS_WARN_STREAM("x,y,z: "<< row[0] << " "
                                 << row[1] << " "
                                 << row[2] << " ");
        
    }


    std::vector<geometry_msgs::Transform> tf_list;
    geometry_msgs::Transform p1;

    // p1.translation.x = 0;
    // p1.translation.y =0.05;
    // tf_list.push_back(p1);

    // p1.translation.x +=0.05;
    // tf_list.push_back(p1);

    p1.translation.z += -5;
    tf_list.push_back(p1);

    p1.translation.z += 10;
    tf_list.push_back(p1);

    arm_interface.moveToFramePTP("home",0.005,M_PI/2,EE_LINKS::PANDA_HAND_BOTTOM);
    gripper_interface.closeGripper();

    arm_interface.do_trajectory(wannabe_g_code_fn("home", tf_list));


}

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


    txt_from_robot(arm_interface,gripper_interface,"thi_robothix_grand_challenge/dyi/dyi_node.cpp");


    arm_interface.moveToHome();
    ROS_INFO("Finished all tasks.");

    return 0;
}
