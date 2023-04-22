#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>


void poseCallback(const geometry_msgs::Pose& msg){

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    // br.sendTransform((msg.position.x, msg.position.y, msg.position.z),
    //                  tf::Transform.quaternion_from_euler(0, 0, msg.orientation.z),
    //                  ros::Time::now(),
    //                  "panda_link_8",
    //                  "panda_EE");
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "box_position_publisher_node");

    return 0;
};