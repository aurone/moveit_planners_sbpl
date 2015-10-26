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
    m_action_set(),
    m_distance_field(),
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
        res.planning_time_ = 0.0;
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    ROS_INFO("Successfully initialized SBPL");

    moveit_msgs::PlanningScenePtr scene_msg(new moveit_msgs::PlanningScene);

    planning_scene::PlanningSceneConstPtr planning_scene = getPlanningScene();
    assert(planning_scene.get()); // otherwise initialization would have failed

    planning_scene->getPlanningSceneMsg(*scene_msg);
    const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();

    moveit_msgs::GetMotionPlan::Request req_msg;
    if (!translateRequest(req_msg)) {
        ROS_WARN("Unable to translate Motion Plan Request to SBPL Motion Plan Request");
        return false;
    }

    moveit_msgs::GetMotionPlan::Response res_msg;
    bool result = m_planner->solve(scene_msg, req_msg, res_msg);
    if (result) {
        moveit::core::RobotModelConstPtr robot_model =
                planning_scene->getRobotModel();

        moveit::core::RobotState ref_state(robot_model);
        robot_state::RobotStatePtr start_state =
                planning_scene->getCurrentStateUpdated(req.start_state);

        robot_trajectory::RobotTrajectoryPtr traj(
                new robot_trajectory::RobotTrajectory(
                        robot_model, getGroupName()));
        traj->setRobotTrajectoryMsg(
                *start_state, res_msg.motion_plan_response.trajectory);

        // res_msg
        //   motion_plan_response
        //     trajectory_start
        //     group_name
        //     trajectory
        //     planning_time
        //     error_code

        res.trajectory_ = traj;
        res.planning_time_ = res_msg.motion_plan_response.planning_time;
        res.error_code_ = res_msg.motion_plan_response.error_code;
    }
    return result;
}

bool SBPLPlanningContext::solve(
    planning_interface::MotionPlanDetailedResponse& res)
{
    ROS_INFO("SBPLPlanningContext::solve(planning_interface::MotionPlanDetailedResponse&)");
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

    const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();

    const std::string& planning_frame =
            req.workspace_parameters.header.frame_id;

    ////////////////
    // Robot Model
    ////////////////

    if (!m_robot_model.init(planning_scene, getGroupName(), planning_frame)) {
        why = "Failed to initialize sbpl Robot Model from moveit Robot Model";
        return false;
    }

    if (!m_robot_model.planningTipLink()) {
        ROS_ERROR("SBPL Plugin does not currently support joint groups without a tip link");
        return false;
    }

    ////////////////////
    // Collision Model
    ////////////////////

    if (!m_collision_checker.init(&m_robot_model, planning_scene)) {
        why = "Failed to initialize sbpl Collision Checker "
                "from Planning Scene and Robot Model";
        return false;
    }

    ///////////////////////////////
    // Action Set Initialization
    ///////////////////////////////

    for (size_t vind = 0; vind < m_robot_model.activeVariableCount(); ++vind) {
        std::vector<double> mprim(m_robot_model.activeVariableCount(), 0.0);
        mprim[vind] = 1.0;
        m_action_set.addMotionPrim(mprim, true, true);
    }

    /////////////////////////////////
    // Distance Field Initalization
    /////////////////////////////////

    // TODO: find the transform from the workspace to the planning frame and
    // use as the origin

    // create a distance field in the planning frame that represents the
    // workspace boundaries

    const double size_x =
        req.workspace_parameters.max_corner.x -
        req.workspace_parameters.min_corner.x;
    const double size_y =
        req.workspace_parameters.max_corner.y -
        req.workspace_parameters.min_corner.y;
    const double size_z =
        req.workspace_parameters.max_corner.z -
        req.workspace_parameters.min_corner.z;

    // TODO: need to either plan in the workspace frame here rather than the
    // common joint root...or need to expand the size of the distance field to
    // match the axis-aligned bb of the workspace and then fill cells outside of
    // the workspace

    const double res_m = 0.02;
    const double max_distance = 0.2;
    const bool propagate_negative_distances = false;

    // get the transform from the workspace to the planning frame
    const std::string& workspace_frame = req.workspace_parameters.header.frame_id;
    if (!planning_scene->knowsFrameTransform(workspace_frame) ||
        !planning_scene->knowsFrameTransform(m_robot_model.planningFrame()))
    {
        ROS_WARN("Planning scene is unable to look transforms for frames '%s' and '%s'", workspace_frame.c_str(), planning_frame.c_str());
        return false;
    }

    // get the transform from the planning frame to the workspace frame
    const Eigen::Affine3d& T_scene_workspace = planning_scene->getFrameTransform(workspace_frame);
    const Eigen::Affine3d& T_scene_planning = planning_scene->getFrameTransform(planning_frame);
    Eigen::Affine3d T_planning_workspace = T_scene_planning.inverse() * T_scene_workspace;

    Eigen::Vector3d workspace_pos_in_planning(T_planning_workspace.translation());

    m_distance_field.reset(new distance_field::PropagationDistanceField(
            size_x, size_y, size_x,
            res_m,
            workspace_pos_in_planning.x(),
            workspace_pos_in_planning.y(),
            workspace_pos_in_planning.z(),
            max_distance,
            propagate_negative_distances));

    // TODO: add octomap and collision objects to the distance field

    //////////////////////////////////////////////
    // SBPL Arm Planner Interface Initialization
    //////////////////////////////////////////////

    m_planner.reset(new sbpl_arm_planner::SBPLArmPlannerInterface(
            &m_robot_model,
            &m_collision_checker,
            &m_action_set,
            m_distance_field.get()));

    sbpl_arm_planner::PlanningParams params;

    std::vector<int> discretization(m_robot_model.activeVariableCount(), 1);
    std::vector<double> deltas(m_robot_model.activeVariableCount());
    for (size_t vind = 0; vind < deltas.size(); ++vind) {
        deltas[vind] = (2.0 * M_PI) / (double)discretization[vind];
    }

    params.num_joints_ = m_robot_model.activeVariableCount();
    params.planning_frame_ = m_robot_model.planningFrame();
    params.group_name_ = m_robot_model.planningGroupName();
    params.planner_name_ = getMotionPlanRequest().planner_id;
    params.planning_joints_ = m_robot_model.planningVariableNames();
    params.coord_vals_ = discretization;
    params.coord_delta_ = deltas;

    if (!m_planner->init(params)) {
        why = "Failed to initialize SBPL Arm Planner Interface";
        return false;
    }

    return true;
}

bool SBPLPlanningContext::translateRequest(
    moveit_msgs::GetMotionPlan::Request& req)
{
    // TODO: translate goal position constraints into planning frame
    // TODO: translate goal orientation constraints into planning frame

    req.motion_plan_request = getMotionPlanRequest();
    return true;
}

} // namespace sbpl_interface
