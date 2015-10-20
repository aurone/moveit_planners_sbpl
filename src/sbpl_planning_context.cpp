#include "sbpl_planning_context.h"

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>

namespace sbpl_interface {

SBPLPlanningContext::SBPLPlanningContext(
    const std::string& name,
    const std::string& group)
:
    Base(name, group)
{
//    moveit_msgs::WorkspaceParameters workspace_parameters;
//    moveit_msgs::RobotState start_state;
//    moveit_msgs::Constraints[] goal_constraints;
//    moveit_msgs::Constraints path_constraints;
//    moveit_msgs::TrajectoryConstraints trajectory_constraints;
//    string planner_id;
//    string group_name;
//    int num_planning_attempts;
//    double allowed_planning_time;
//    double max_velocity_scaling_factor;
}

SBPLPlanningContext::~SBPLPlanningContext()
{
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
    return false;
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
    return false;
}

bool SBPLPlanningContext::terminate()
{
    return true;
}

void SBPLPlanningContext::clear()
{
}

} // namespace sbpl_interface
