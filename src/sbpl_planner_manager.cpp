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

    SBPLPlanningContext* sbpl_context =
            new SBPLPlanningContext("sbpl_planning_context", req.group_name);

    sbpl_context->setPlanningScene(planning_scene);

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

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbpl_interface::SBPLPlannerManager, planning_interface::PlannerManager);
