#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <geometry_msgs/PoseStamped.h> 



#include <tf2_ros/transform_listener.h>



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





    // // collision object

    // moveit_msgs::CollisionObject collision_object;
    // collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
    // collision_object.id = "box1";

    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[primitive.BOX_X] = 0.5;
    // primitive.dimensions[primitive.BOX_Y] = 0.05;
    // primitive.dimensions[primitive.BOX_Z] = 0.5;

    // geometry_msgs::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    // box_pose.position.x = 0.5;
    // box_pose.position.y = 0.0;
    // box_pose.position.z = 0.15;

    // collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.operation = collision_object.ADD;

    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(collision_object);


    // ROS_INFO_NAMED("tutorial", "Add an object into the world");
    // planning_scene_interface.addCollisionObjects(collision_objects);


    // // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    // // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
              move_group_interface_arm.getJointModelGroupNames().end(), 
              std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;


    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("ready"));
    move_group_interface_arm.setMaxVelocityScalingFactor(0.1);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.5);
    move_group_interface_arm.move();



      geometry_msgs::PoseStamped current_pose;
      current_pose = move_group_interface_arm.getCurrentPose("panda_hand_tcp");


    while(ros::ok())
    {

     
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("panda_link0", "fiducial_6",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

      ROS_INFO_STREAM("Transform: " << transformStamped); 









      ROS_INFO_STREAM("current pose: " << current_pose); 

      geometry_msgs::Pose target_pose1;
    
      target_pose1.orientation  = current_pose.pose.orientation;
      target_pose1.position.x   = transformStamped.transform.translation.x;
      target_pose1.position.y   = transformStamped.transform.translation.y;
      target_pose1.position.z   = transformStamped.transform.translation.z + 0.35;

      // target_pose1.orientation = current_pose.pose.orientation; 
      // target_pose1.position.x  = 0.4; 
      // target_pose1.position.y  = 0.0; 
      // target_pose1.position.z  = 0.2; 
      move_group_interface_arm.setPoseTarget(target_pose1);

      const auto success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


      move_group_interface_arm.move();
      




      // // 1. Move to home position
      // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("ready"));
      // move_group_interface_arm.setMaxVelocityScalingFactor(1.0);
      // move_group_interface_arm.setMaxAccelerationScalingFactor(0.5);
      



      // bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      // const auto home_pose = move_group_interface_arm.getCurrentPose("panda_hand_tcp");
      // visual_tools.publishAxisLabeled(home_pose.pose, "HOME");
      // visual_tools.publishTrajectoryLine(my_plan_arm.trajectory_, joint_model_group);
      // visual_tools.trigger();
      // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the real robot");

      // move_group_interface_arm.move();





      // // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
      // geometry_msgs::PoseStamped current_pose;
      // current_pose = move_group_interface_arm.getCurrentPose("panda_hand_tcp");

      // geometry_msgs::Pose target_pose1;
    
      // target_pose1.orientation  = current_pose.pose.orientation;
      // target_pose1.position.x   = current_pose.pose.position.x + 0.1;
      // target_pose1.position.y   = current_pose.pose.position.y + 0.2;
      // target_pose1.position.z   = current_pose.pose.position.z + 0.2;
      // move_group_interface_arm.setPoseTarget(target_pose1);

      // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


      // visual_tools.publishAxisLabeled(target_pose1, "pose1");
      // visual_tools.publishTrajectoryLine(my_plan_arm.trajectory_, joint_model_group);
      // visual_tools.trigger();
      // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the real robot");


      // move_group_interface_arm.move();












      // // 3. Open the gripper
      //   moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
      // move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
      // move_group_interface_gripper.setMaxVelocityScalingFactor(1.0);
      // move_group_interface_gripper.setMaxAccelerationScalingFactor(0.6);

      // success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      // move_group_interface_gripper.move();










      // // 4. Move the TCP close to the object
      // target_pose1.position.z = target_pose1.position.z - 0.3;

      // // target_pose1.position.x   = current_pose.pose.position.x  0.4;
      // // target_pose1.position.y   = current_pose.pose.position.y - 0.4;
      // // target_pose1.position.z   = current_pose.pose.position.z + 0.3;

      // move_group_interface_arm.setPoseTarget(target_pose1);

      // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


      // visual_tools.publishAxisLabeled(target_pose1, "pose2");
      // visual_tools.publishTrajectoryLine(my_plan_arm.trajectory_, joint_model_group);
      // visual_tools.trigger();
      // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the real robot");

      // move_group_interface_arm.move();








      // // 4. Move the TCP close to the object
      // // target_pose1.position.z = target_pose1.position.z - 0.2;

      // // target_pose1.position.x   = current_pose.pose.position.x  0.4;
      // target_pose1.position.y   = current_pose.pose.position.y - 0.4;
      // // target_pose1.position.z   = current_pose.pose.position.z + 0.3;

      // move_group_interface_arm.setPoseTarget(target_pose1);

      // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


      // visual_tools.publishAxisLabeled(target_pose1, "pose3");
      // visual_tools.publishTrajectoryLine(my_plan_arm.trajectory_, joint_model_group);
      // visual_tools.trigger();
      // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the real robot");

      // move_group_interface_arm.move();










      // // 5. Close the  gripper
      // move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));

      // success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      // move_group_interface_gripper.move();

    }



    // // 6. Move the TCP above the plate
    // target_pose1.position.z = target_pose1.position.z + 0.2;
    // target_pose1.position.x = target_pose1.position.x - 0.6;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // // 7. Lower the TCP above the plate
    // target_pose1.position.z = target_pose1.position.z - 0.14;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // 8. Open the gripper
    // move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    // success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_gripper.move();

  ros::shutdown();
  return 0;
}
