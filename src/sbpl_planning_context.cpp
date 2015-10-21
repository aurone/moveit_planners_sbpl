#include "sbpl_planning_context.h"

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

namespace sbpl_interface {

SBPLPlanningContext::SBPLPlanningContext(
    const std::string& name,
    const std::string& group)
:
    Base(name, group),
    m_robot_model(),
    m_collision_checker(),
    m_action_set("No, thank you"),
    m_distance_field(1.0, 1.0, 1.0, 0.02, 0.0, 0.0, 0.0, 0.2, false),
    m_planner()
{
    ROS_INFO("Constructed SBPL Planning Context");
}

SBPLPlanningContext::~SBPLPlanningContext()
{
    ROS_INFO("Destructed SBPL Planning Context");
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
    ROS_INFO("SBPLPlanningContext::solve()");

    std::string why;
    if (!initSBPL(why)) {
        ROS_WARN("Failed to initialize SBPL (%s)", why.c_str());
        return false; 
    }

    moveit_msgs::PlanningScenePtr scene_msg(new moveit_msgs::PlanningScene);

    planning_scene::PlanningSceneConstPtr planning_scene = getPlanningScene();
    assert(planning_scene.get()); // otherwise initialization would have failed

    planning_scene->getPlanningSceneMsg(*scene_msg);
    const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();
    moveit_msgs::GetMotionPlan::Request req_msg;
    req_msg.motion_plan_request = req;

    moveit_msgs::GetMotionPlan::Response res_msg;
    bool result = m_planner->solve(scene_msg, req_msg, res_msg);
    if (result) {
        // TODO: convert moveit_msgs::GetMotionPlan::Response to
        // planning_interface::MotionPlanResponse
//        res = res_msg.motion_plan_response;
    }
    return result;
}

bool SBPLPlanningContext::solve(
    planning_interface::MotionPlanDetailedResponse& res)
{
    ROS_INFO("SBPLPlanningContext::solve()");
    return false;
}

bool SBPLPlanningContext::terminate()
{
    ROS_INFO("SBPLPlanningContext::terminate()");
    return true;
}

void SBPLPlanningContext::clear()
{
    ROS_INFO("SBPLPlanningContext::clear()");
}

bool SBPLPlanningContext::initSBPL(std::string& why)
{
    planning_scene::PlanningSceneConstPtr planning_scene = getPlanningScene();
    if (!planning_scene) {
        why = "No Planning Scene available";
        return false;
    }

    moveit::core::RobotModelConstPtr robot_model;
    robot_model = planning_scene->getRobotModel();
    if (!robot_model) {
        why = "No robot model available via planning scene";
        return false;
    }

    if (!m_robot_model.init(robot_model, getGroupName())) {
        why = "Failed to initialize sbpl Robot Model from moveit Robot Model";
        return false;
    }

    if (!m_collision_checker.init(&m_robot_model, planning_scene)) {
        why = "Failed to initialize sbpl Collision Checker "
                "from Planning Scene and Robot Model";
        return false;
    }

    m_planner.reset(new sbpl_arm_planner::SBPLArmPlannerInterface(
            &m_robot_model,
            &m_collision_checker,
            &m_action_set,
            &m_distance_field));

    if (!m_planner->init()) {
        why = "Failed to initialize SBPL Arm Planner Interface";
        return false;
    }

    return true;
}

} // namespace sbpl_interface
