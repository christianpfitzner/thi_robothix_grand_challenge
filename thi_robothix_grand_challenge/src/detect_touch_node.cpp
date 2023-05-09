

// create a ros node to detect the forces from the franka robot


#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/JointState.h>




// callback function for the force sensor
void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // get the force values
    std::vector<double> force_values = msg->effort;

    // get the joint names
    std::vector<std::string> joint_names = msg->name;

    // get the joint positions
    std::vector<double> joint_positions = msg->position;

    // get the joint velocities
    std::vector<double> joint_velocities = msg->velocity;

    // get the joint efforts
    std::vector<double> joint_efforts = msg->effort;



    if(abs(joint_efforts[0]) > 3.0)
    {
        ROS_INFO("TI touched the robot");
    }
    else
    {
        ROS_INFO("TI did not touch the robot");
    }
}





// write the main ros function

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_touch_node");
    ros::NodeHandle nh;

    // create a subscriber to the force sensor
    ros::Subscriber force_sub = nh.subscribe("/joint_states", 1, joint_state_callback);



    


    ros::spin(); 
}
