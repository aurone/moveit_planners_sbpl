#ifndef sbpl_interface_SBPLPlanningContext_h
#define sbpl_interface_SBPLPlanningContext_h

#include <memory>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/OrientedBoundingBox.h>
#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>

#include "moveit_robot_model.h"
#include "moveit_collision_checker.h"

namespace sbpl_interface {

class SBPLPlanningContext : public planning_interface::PlanningContext
{
public:

    typedef planning_interface::PlanningContext Base;

    SBPLPlanningContext(const std::string& name, const std::string& group);
    virtual ~SBPLPlanningContext();

    virtual bool solve(planning_interface::MotionPlanResponse& res);
    virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);
    virtual bool terminate();
    virtual void clear();

private:

    // sbpl planner components
    MoveItRobotModel m_robot_model;
    MoveItCollisionChecker m_collision_checker;
    sbpl_arm_planner::ActionSet m_action_set;
    std::unique_ptr<distance_field::PropagationDistanceField> m_distance_field;

    std::unique_ptr<sbpl_arm_planner::SBPLArmPlannerInterface> m_planner;

    /// \brief Initialize SBPL constructs
    /// \param[out] Reason for failure if initialization is unsuccessful
    /// \return true if successful; false otherwise
    bool initSBPL(std::string& why);

    bool translateRequest(moveit_msgs::GetMotionPlan::Request& req);

    bool getPlanningFrameWorkspaceAABB(
        const moveit_msgs::WorkspaceParameters& workspace,
        const planning_scene::PlanningScene& scene,
        moveit_msgs::OrientedBoundingBox& aabb);
};

MOVEIT_CLASS_FORWARD(SBPLPlanningContext);

} // namespace sbpl_interface

#endif
