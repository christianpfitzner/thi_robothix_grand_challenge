#include "ros/ros.h"
#include "moveit_msgs/MotionPlanRequest.h"
#include "moveit_msgs/MotionPlanResponse.h"
#include "moveit/kinematic_constraints/kinematic_constraint.h"
#include "moveit/kinematic_constraints/utils.h"

class PilzMoveItInterface
{

    private:
    
    moveit_msgs::MotionPlanResponse plan_res;

    public:

    moveit_msgs::MotionPlanRequest do_plan_request(double max_velocity_scaling_factor, double max_acceleration_scaling_factor, 
                            double max_cartesian_speed, std::string frame_id, geometry_msgs::PoseStamped& target_pose)
    {
        moveit_msgs::Constraints pose_start_goal = kinematic_constraints::constructGoalConstraints("panda_link0",target_pose, 0.01, 0);

        //pose_start_goal.name = frame_id;

        std::vector<moveit_msgs::Constraints> constraints;
        constraints.push_back(pose_start_goal);

        ROS_WARN("Planning Req in progress 1");
        
        moveit_msgs::MotionPlanRequest plan_req;

        plan_req.pipeline_id = "pilz_industrial_motion_planner";
        //plan_req.pipeline_id = "chomp_motion_planner";
        plan_req.planner_id = "LIN"; 
        plan_req.group_name = "panda_hand";

        //set velocitiers and acc
        plan_req.max_velocity_scaling_factor = max_velocity_scaling_factor;
        plan_req.max_acceleration_scaling_factor = max_acceleration_scaling_factor;
        plan_req.max_cartesian_speed = max_cartesian_speed;
        //plan_req.cartesian_speed_limited_link = "panda_hand_tcp";
        

        plan_req.allowed_planning_time = 5;
        plan_req.num_planning_attempts = 20;
        //set goal constrains
        plan_req.goal_constraints = constraints;

        ROS_WARN("Planning Req made");


        return plan_req;
    }

};