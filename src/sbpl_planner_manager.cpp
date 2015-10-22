#include "sbpl_planner_manager.h"

#include <moveit/planning_scene/planning_scene.h>

#include "sbpl_planning_context.h"

namespace sbpl_interface {

const std::string DefaultPlanningAlgorithm = "ARA*";

SBPLPlannerManager::SBPLPlannerManager() :
    Base()
{
    ROS_INFO("Constructed SBPL Planner Manager");
}

SBPLPlannerManager::~SBPLPlannerManager()
{
    ROS_INFO("Destructed SBPL Planner Manager");
}

bool SBPLPlannerManager::initialize(
    const robot_model::RobotModelConstPtr& model,
    const std::string& ns)
{
    ROS_INFO("Initialized SBPL Planner Manager");
    m_robot_model = model;
    m_ns = ns;
    return true;
}

std::string SBPLPlannerManager::getDescription() const
{
    return "Search-Based Planning Algorithms";
}

void SBPLPlannerManager::getPlanningAlgorithms(
    std::vector<std::string>& algs) const
{
    algs.push_back("ARA*");
}

planning_interface::PlanningContextPtr SBPLPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    planning_interface::PlanningContextPtr context;

    if (!planning_scene) {
        ROS_WARN("Planning Context is null");
        return context;
    }

    logPlanningScene(*planning_scene);
    logMotionRequest(req);

    SBPLPlanningContext* sbpl_context =
            new SBPLPlanningContext("sbpl_planning_context", req.group_name);

    sbpl_context->setPlanningScene(planning_scene);
    sbpl_context->setMotionPlanRequest(req);

    context.reset(sbpl_context);
    return context;
}

bool SBPLPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    ROS_INFO("SBPLPlannerManager::canServiceRequest()");
    return false;
}

void SBPLPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pcs)
{
    ROS_INFO("SBPLPlannerManager::setPlannerConfigurations");
}

void SBPLPlannerManager::terminate() const
{
    ROS_INFO("SBPLPlannerManager::terminate()");
}

void SBPLPlannerManager::logPlanningScene(
    const planning_scene::PlanningScene& scene) const
{
    ROS_INFO("Planning Scene");
    ROS_INFO("    Name: %s", scene.getName().c_str());
    ROS_INFO("    Has Parent: %s", scene.getParent() ? "true" : "false");
    ROS_INFO("    Has Robot Model: %s", scene.getRobotModel() ? "true" : "false");
    ROS_INFO("    Planning Frame: %s", scene.getPlanningFrame().c_str());
    ROS_INFO("    Active Collision Detector Name: %s", scene.getActiveCollisionDetectorName().c_str());
    ROS_INFO("    Has World: %s", scene.getWorld() ? "true" : "false");
    ROS_INFO("    Has Collision Robot: %s", scene.getCollisionRobot() ? "true" : "false");
}

void SBPLPlannerManager::logMotionRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    ROS_INFO("Motion Plan Request");
    ROS_INFO("  workspace_parameters");
    ROS_INFO("    header");
    ROS_INFO_STREAM("      seq: " << req.workspace_parameters.header.seq);
    ROS_INFO_STREAM("      stamp: " << req.workspace_parameters.header.stamp);
    ROS_INFO_STREAM("      frame_id: " << req.workspace_parameters.header.frame_id.c_str());
    ROS_INFO("    min_corner");
    ROS_INFO_STREAM("      x: " << req.workspace_parameters.min_corner.x);
    ROS_INFO_STREAM("      y: " << req.workspace_parameters.min_corner.y);
    ROS_INFO_STREAM("      z: " << req.workspace_parameters.min_corner.z);
    ROS_INFO("    max_corner");
    ROS_INFO_STREAM("      x: " << req.workspace_parameters.max_corner.x);
    ROS_INFO_STREAM("      y: " << req.workspace_parameters.max_corner.y);
    ROS_INFO_STREAM("      z: " << req.workspace_parameters.max_corner.z);
    ROS_INFO("  start_state");
    ROS_INFO("  goal_constraints");
    ROS_INFO("  path_constraints");
    ROS_INFO("  trajectory_constraints");
    ROS_INFO_STREAM("  planner_id: " << req.planner_id);
    ROS_INFO_STREAM("  group_name: " << req.group_name);
    ROS_INFO_STREAM("  num_planning_attempts: " << req.num_planning_attempts);
    ROS_INFO_STREAM("  allowed_planning_time: " << req.allowed_planning_time);
    ROS_INFO_STREAM("  max_velocity_scaling_factor: " << req.max_velocity_scaling_factor);
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbpl_interface::SBPLPlannerManager, planning_interface::PlannerManager);
