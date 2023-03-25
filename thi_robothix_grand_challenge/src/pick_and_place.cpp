#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <tf2_ros/transform_listener.h>



#include <geometry_msgs/PoseStamped.h> 


moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;





bool moveToFrame( moveit::planning_interface::MoveGroupInterface* mgi, std::string frame_id)
{
  bool success = false; 

  geometry_msgs::TransformStamped transformStamped;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);  

  ros::Duration(1.0).sleep();
  try
  {
    transformStamped = tfBuffer.lookupTransform("panda_link0", frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();

    return success; 
  }

  ROS_ERROR_STREAM(transformStamped); 


geometry_msgs::PoseStamped target_pose; 


target_pose.header.frame_id = "panda_link0";

target_pose.pose.position.x    = transformStamped.transform.translation.x; 
target_pose.pose.position.y    = transformStamped.transform.translation.y; 
target_pose.pose.position.z    = transformStamped.transform.translation.z; 

target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
target_pose.pose.orientation.x = transformStamped.transform.rotation.x;  
target_pose.pose.orientation.y = transformStamped.transform.rotation.y;  
target_pose.pose.orientation.z = transformStamped.transform.rotation.z;  

mgi->setPoseTarget(target_pose);



success = (mgi->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


// visual_tools.publishAxisLabeled(target_pose_rough_localization.pose, "pose1");
// visual_tools.publishTrajectoryLine(my_plan_arm.trajectory_, joint_model_group);
// visual_tools.trigger();

if(success) 
  mgi->move();





  return success; 
}






int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM     = "panda_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "panda_hand";
    


  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
  const robot_state::JointModelGroup* joint_model_group = move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;




  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();



    ROS_INFO_NAMED("tutorial", "Planning frame: %s",    move_group_interface_arm.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "Planning frame: %s",    move_group_interface_gripper.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface_arm.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface_gripper.getEndEffectorLink().c_str());

    while(ros::ok())
    {

      // We can get a list of all the groups in the robot:
      std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
                move_group_interface_arm.getJointModelGroupNames().end(), 
                std::ostream_iterator<std::string>(std::cout, ", "));

      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      


      bool success = false; 

      // 1. Move to home position
      move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("ready"));



      move_group_interface_arm.setMaxVelocityScalingFactor(0.2);
      move_group_interface_arm.setMaxAccelerationScalingFactor(0.5);
      

      geometry_msgs::PoseStamped current_pose;
      current_pose = move_group_interface_arm.getCurrentPose("panda_hand_tcp");
      success      = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
      move_group_interface_arm.move();




      // Move the arm above the target for rough localization
 


      // geometry_msgs::PoseStamped target_pose_rough_localization; 
      // target_pose_rough_localization.header.frame_id = "panda_link0";

      // // target_pose1.orientation  = current_pose.pose.orientation;
      // target_pose_rough_localization.pose.position.x   = 0.434;
      // target_pose_rough_localization.pose.position.y   = 0.195;
      // target_pose_rough_localization.pose.position.z   = 0.394;


      // tf2_ros::Buffer tfBuffer;
      // tf2_ros::TransformListener tfListener(tfBuffer);  


      moveToFrame(&move_group_interface_arm, "inspect_box" ); 

        // geometry_msgs::TransformStamped transformStamped;
        // ros::Duration(1.0).sleep();
        // try
        // {
        //   transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_link8", ros::Time(0));
        // }
        // catch (tf2::TransformException &ex) 
        // {
        //   ROS_WARN("%s",ex.what());
        //   ros::Duration(1.0).sleep();
        //   continue;
        // }


      // transformStamped.transform.rotation

      // target_pose_rough_localization.pose.orientation = transformStamped.transform.rotation; 

      // move_group_interface_arm.setPoseTarget(target_pose_rough_localization);



      // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


      // visual_tools.publishAxisLabeled(target_pose_rough_localization.pose, "pose1");
      // visual_tools.publishTrajectoryLine(my_plan_arm.trajectory_, joint_model_group);
      // visual_tools.trigger();
      // // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the real robot");

      // if(success) 
      //   move_group_interface_arm.move();



      move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
      move_group_interface_gripper.setMaxVelocityScalingFactor(0.3);
      move_group_interface_gripper.setMaxAccelerationScalingFactor(0.6);
      success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(success)
        move_group_interface_gripper.move();
      else  
        ROS_ERROR_STREAM("Error"); 




      geometry_msgs::PoseStamped pose;
      pose.pose.position.y = -0.05;
      pose.pose.orientation.w = 1;
      pose.header.frame_id = "panda_hand_tcp"; // move_group_interface_arm.getEndEffectorLink();
      move_group_interface_arm.setPoseTarget(pose);

      success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      if(success) 
        move_group_interface_arm.move();







      move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));
      move_group_interface_gripper.setMaxVelocityScalingFactor(0.3);
      move_group_interface_gripper.setMaxAccelerationScalingFactor(0.6);
      success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if(success) 
        move_group_interface_gripper.move();
      

    }


  ros::shutdown();
  return 0;
}
