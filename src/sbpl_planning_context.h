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

#include <moveit_planners_sbpl/moveit_robot_model.h>

#include "moveit_collision_checker.h"

namespace sbpl_interface {

class SBPLPlanningContext : public planning_interface::PlanningContext
{
public:

    typedef planning_interface::PlanningContext Base;

    /// \brief Construct an SBPL Planning Context with a pre-constructed
    SBPLPlanningContext(
        MoveItRobotModel* robot_model,
        const std::string& name,
        const std::string& group);

    virtual ~SBPLPlanningContext();

    /// \sa planning_interface::PlanningContext::solve(planning_interface::MotionPlanResponse&)
    virtual bool solve(planning_interface::MotionPlanResponse& res);

    /// \sa planning_interface::PlanningContext::solve(planning_interface::MotionPlanDetailedResponse&)
    virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

    /// \sa planning_interface::PlanningContext::terminate
    virtual bool terminate();

    /// \sa planning_interface::PlanningContext::clear
    virtual void clear();

    /// \brief Initialize the SBPL Planning Context
    ///
    /// This phase of initialization only initializes structures that do not
    /// depend on having an active PlanningScene or MotionPlanRequest. A second
    /// phase of initialization happens on the first call to solve() which will
    /// initialize structures dependant on either of these things.
    ///
    /// The MoveItRobotModel passed to the constructor must be initialized
    /// before this initialization is possible.
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

    double m_epsilon;

    bool m_shortcut_path;
    bool m_interpolate_path;

    bool m_use_bfs_heuristic;
    double m_bfs_res_x;
    double m_bfs_res_y;
    double m_bfs_res_z;
    double m_bfs_sphere_radius;

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
