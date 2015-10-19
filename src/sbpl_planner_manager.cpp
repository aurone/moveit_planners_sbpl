#include "sbpl_planner_manager.h"

namespace sbpl_interface {

SBPLPlannerManager::SBPLPlannerManager() :
    Base()
{
}

SBPLPlannerManager::~SBPLPlannerManager()
{
}

bool SBPLPlannerManager::initialize(
    const robot_model::RobotModelConstPtr& model,
    const std::string& ns)
{
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
}

bool SBPLPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    return false;
}

void SBPLPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pcs)
{
}

void SBPLPlannerManager::terminate() const
{
}

} // namespace sbpl_interface
