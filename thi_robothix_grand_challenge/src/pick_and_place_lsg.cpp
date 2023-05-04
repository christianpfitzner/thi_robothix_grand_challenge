#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h> 

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>


moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;


bool moveToPose(moveit::planning_interface::MoveGroupInterface* mgi, geometry_msgs::PoseStamped &pose)
{
  bool success = false; 
  mgi->setPoseTarget(pose);
  success = (mgi->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success) mgi->move();
  return success; 
}

bool moveToFrameLIN(moveit::planning_interface::MoveGroupInterface* mgi, std::string frame_id)
{

    geometry_msgs::PosGraspmgi->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    bool success = (mgi->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success) mgi->move();

    return success;

}

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
  catch (tf2::TranGrasp
  geometry_msgs::PoseStamped target_pose; 
  target_pose.header.frame_id    = "panda_link0";

  target_pose.pose.position.x    = transformStamped.transform.translation.x; 
  target_pose.pose.position.y    = transformStamped.transform.translation.y; 
  target_pose.pose.position.z    = transformStamped.transform.translation.z; 

  target_pose.pose.orientation.w = transformStamped.transform.rotation.w;
  target_pose.pose.orientation.x = transformStamped.transform.rotation.x;  
  target_pose.pose.orientation.y = transformStamped.transform.rotation.y;  
  target_pose.pose.orientation.z = transformStamped.transform.rotation.z;  

  mgi->setPoseTarget(target_pose);

  success = (mgi->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success) mgi->move();

    return success; 
}


bool closeGripper(moveit::planning_interface::MoveGroupInterface* mgi)
{
    mgi->setJointValueTarget(mgi->getNamedTargetValues("close"));

    bool success = (mgi->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success) mgi->move();
      
    return success; 
}



bool openGripper(moveit::planning_interface::MoveGroupInterface* mgi)
{
    mgi->setJointValueTarget(mgi->getNamedTargetValues("open"));

    bool success = (mgi->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success) mgi->move();
      
    return success; 
}


bool setGripperWidth(moveit::planning_interface::MoveGroupInterface* mgi, const double width)
{
    std::vector<double> finger_width; 
    finger_width.resize(2); 
    finger_width[0] = width; 
    finger_width[1] = width; 

    mgi->setJointValueTarget(finger_width);

    const bool success = (mgi->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   
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
  static const std::string PLANNING_GROUP_ARM     = "panda_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "panda_hand";
    


  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
  const robot_state::JointModelGroup* joint_model_group = move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  move_group_interface_arm.setPlanningTime(5.0);
  move_group_interface_gripper.setPlanningTime(5.0);
  // move_group_interface_gripper.

  // set the speed for the gripper
  move_group_interface_gripper.setMaxVelocityScalingFactor(0.5);
  move_group_interface_gripper.setMaxAccelerationScalingFactor(0.5);
  // move_group_interface_gripper.setGoalJointTolerance(0.02);


  // actionlib::SimpleActionClient<franka_gripper::GraspActionGoal> grasp_client("/franka_gripper/grasp", true);
  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
  ac.waitForServer();





  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


    while(ros::ok())
    {

      // We can get a list of all the groups in the robot:
      std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
                move_group_interface_arm.getJointModelGroupNames().end(), 
                std::ostream_iterator<std::string>(std::cout, ", "));

      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      


      bool success = false; 


      move_group_interface_arm.setPoseReferenceFrame("panda_hand_tcp"); 
      move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("ready"));
      move_group_interface_arm.setMaxVelocityScalingFactor(0.2);
      move_group_interface_arm.setMaxAccelerationScalingFactor(0.25);
      

      success      = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
      move_group_interface_arm.move();

      //box
      // moveit_msgs::CollisionObject collision_object;
      // collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
      // collision_object.id = "box1";

      // shape_msgs::SolidPrimitive primitive;
      // primitive.type = primitive.BOX;
      // primitive.dimensions.resize(3);
      // primitive.dimensions[primitive.BOX_X] = 0.5;
      // primitive.dimensions[primitive.BOX_Y] = 0.01;
      // primitive.dimensions[primitive.BOX_Z] = 0.1;

      // geometry_msgs::Pose box_pose;
      // box_pose.orientation.w = 1.0;
      // box_pose.position.x = 0.452;
      // box_pose.position.y = 0.200;
      // box_pose.position.z = 0.122;

      // collision_object.primitives.push_back(primitive);
      // collision_object.primitive_poses.push_back(box_pose);
      // collision_object.operation = collision_object.ADD;

      // std::vector<moveit_msgs::CollisionObject> collision_objects;
      // collision_objects.push_back(collision_object);
      
      // planning_scene_interface.addCollisionObjects(collision_objects);

      //move commands
      
      closeGripper(&move_group_interface_gripper);
      openGripper(&move_group_interface_gripper);


      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);  
      geometry_msgs::TransformStamped transformStamped;
      ros::Duration(1.0).sleep();
      try
      {
        transformStamped = tfBuffer.lookupTransform("panda_link0", "fiducial_0", ros::Time(0));
      }
      catch (tf2::TransformException &ex) 
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();

        return success; 
      }

      geometry_msgs::PoseStamped target; 
      target.header.frame_id    = "panda_link0";
      target.pose.position.x    = transformStamped.transform.translation.x; // + 0.01; 
      target.pose.position.y    = transformStamped.transform.translation.y; 
      target.pose.position.z    = transformStamped.transform.translation.z; // - 0.04; 

      tf2::Quaternion q;
      q.setRPY(3.14, 0, 0);
      target.pose.orientation = tf2::toMsg(q);

      geometry_msgs::PoseStamped p1 = target;
      p1.pose.position.z += 0.20;

      moveToPose(&move_group_interface_arm, p1);





      ros::Duration(1.0).sleep();
      try
      {
        transformStamped = tfBuffer.lookupTransform("panda_link0", "fiducial_0", ros::Time(0));
      }
      catch (tf2::TransformException &ex) 
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();

        return success; 
      }

      // geometry_msgs::PoseStamped target; 
      target.header.frame_id    = "panda_link0";
      target.pose.position.x    = transformStamped.transform.translation.x; // - 0.02; 
      target.pose.position.y    = transformStamped.transform.translation.y - 0.005 ; 
      target.pose.position.z    = transformStamped.transform.translation.z + 0.05; 

      tf2::Quaternion r;
      r.setW(transformStamped.transform.rotation.w);
      r.setX(transformStamped.transform.rotation.x);
      r.setY(transformStamped.transform.rotation.y);
      r.setZ(transformStamped.transform.rotation.z);

      q.setRPY(3.14, 0, 0);
      q = q * r;
      q.normalize();
      target.pose.orientation = tf2::toMsg(q);

      moveToPose(&move_group_interface_arm, target);
      
      target.pose.position.z    = transformStamped.transform.translation.z - 0.01; 
      moveToPose(&move_group_interface_arm, target);





      franka_gripper::GraspGoal grasp_action; 
      grasp_action.width = 0.065*0.5; 
      grasp_action.speed = 1.0; 
      grasp_action.force = 6;  // N
      grasp_action.epsilon.inner = 0.01;  // Maximum tolerated deviation when the actual grasped width is
      grasp_action.epsilon.outer = 0.01;

      ac.sendGoal(grasp_action); 

      ac.waitForResult(ros::Duration(4.0)); 
      if(ac.getResult()->success)
      {
        ROS_ERROR_STREAM("yeah!!!!");
      }

      target.pose.position.z    = transformStamped.transform.translation.z + 0.2; 
      moveToPose(&move_group_interface_arm, target);


      // - Translation: [0.015, -0.597, 0.000]
      // - Rotation: in Quaternion [0.718, -0.696, 0.001, 0.018]
      //       in RPY (radian) [3.116, -0.026, -1.540]
      //       in RPY (degree) [178.558, -1.499, -88.208]

      target.header.frame_id    = "panda_link0";
      target.pose.position.x    = 0.015; 
      target.pose.position.y    = -0.597; 
      target.pose.position.z    = 0; 

      // tf2::Quaternion q;
      q.setRPY(3.116, -0.026, -1.540);
      target.pose.orientation = tf2::toMsg(q);

      moveToPose(&move_group_interface_arm, target);
      openGripper(&move_group_interface_gripper);

      // setGripperWidth(&move_group_interface_gripper, 0.064);   
      // moveToPose(&move_group_interface_arm, p1);
      // openGripper(&move_group_interface_gripper); 
      
      // openGripper(    &move_group_interface_gripper); 

      // moveToFrame(&move_group_interface_arm, "corner" ); 
      // openGripper(    &move_group_interface_gripper); 
      // moveToFrame(&move_group_interface_arm, "totmann" ); 
      // moveToFrame(&move_group_interface_arm, "camera" ); 
      // moveToFrame(&move_group_interface_arm, "corner" ); 
      // moveToFrame(    &move_group_interface_arm, "place" ); 
      // openGripper(    &move_group_interface_gripper); 
      // moveToFrame(    &move_group_interface_arm, "pre_place" ); 



    }


  ros::shutdown();
  return 0;
}
