#ifndef sbpl_interface_SBPLPlanningContext_h
#define sbpl_interface_SBPLPlanningContext_h

#include <map>
#include <memory>
#include <string>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/OrientedBoundingBox.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>

#include "moveit_robot_model.h"
#include "moveit_collision_checker.h"

namespace sbpl_interface {

class SBPLPlanningContext : public planning_interface::PlanningContext
{
public:

    typedef planning_interface::PlanningContext Base;
    SBPLPlanningContext(
        MoveItRobotModel* robot_model,
        const std::string& name,
        const std::string& group);
    virtual ~SBPLPlanningContext();

    virtual bool solve(planning_interface::MotionPlanResponse& res);
    virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);
    virtual bool terminate();
    virtual void clear();

    bool init(const std::map<std::string, std::string>& config);

private:

    // sbpl planner components
    MoveItRobotModel* m_robot_model;
    MoveItCollisionChecker m_collision_checker;
    sbpl_arm_planner::ActionSet m_action_set;
    std::unique_ptr<distance_field::PropagationDistanceField> m_distance_field;

    std::unique_ptr<sbpl_arm_planner::SBPLArmPlannerInterface> m_planner;

    std::map<std::string, std::string> m_config;

    std::map<std::string, double> m_disc;
    bool m_use_xyz_snap_mprim;
    bool m_use_rpy_snap_mprim;
    bool m_use_xyzrpy_snap_mprim;
    bool m_use_short_dist_mprims;
    double m_xyz_snap_thresh;
    double m_rpy_snap_thresh;
    double m_xyzrpy_snap_thresh;
    double m_short_dist_mprims_thresh;
    bool m_shortcut_path;

    bool m_use_bfs_heuristic;
    double m_bfs_res_x;
    double m_bfs_res_y;
    double m_bfs_res_z;

    /// \brief Initialize SBPL constructs
    /// \param[out] Reason for failure if initialization is unsuccessful
    /// \return true if successful; false otherwise
    bool initSBPL(std::string& why);

    bool translateRequest(moveit_msgs::MotionPlanRequest& req);

    bool getPlanningFrameWorkspaceAABB(
        const moveit_msgs::WorkspaceParameters& workspace,
        const planning_scene::PlanningScene& scene,
        moveit_msgs::OrientedBoundingBox& aabb);

    bool initHeuristicGrid(
        const planning_scene::PlanningScene& scene,
        const moveit_msgs::WorkspaceParameters& workspace);
};

MOVEIT_CLASS_FORWARD(SBPLPlanningContext);

} // namespace sbpl_interface

#endif
