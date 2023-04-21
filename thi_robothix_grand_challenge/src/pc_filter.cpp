

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>




void pc_callback(const sensor_msgs::PointCloud2::ConstPtr msg)
{

    msg->




}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_filter_node"); 



    ros::spin(); 


    return 0; 
}






