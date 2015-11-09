#include "sbpl_planning_context.h"

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>

#include <leatherman/print.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

namespace sbpl_interface {

SBPLPlanningContext::SBPLPlanningContext(
    MoveItRobotModel* robot_model,
    const std::string& name,
    const std::string& group)
:
    Base(name, group),
    m_robot_model(robot_model),
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

    moveit_msgs::MotionPlanRequest req_msg;
    if (!translateRequest(req_msg)) {
        ROS_WARN("Unable to translate Motion Plan Request to SBPL Motion Plan Request");
        return false;
    }

    moveit_msgs::MotionPlanResponse res_msg;
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
                *start_state, res_msg.trajectory);

        // res_msg
        //   motion_plan_response
        //     trajectory_start
        //     group_name
        //     trajectory
        //     planning_time
        //     error_code

        res.trajectory_ = traj;
        res.planning_time_ = res_msg.planning_time;
        res.error_code_ = res_msg.error_code;
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

void SBPLPlanningContext::setPlannerConfiguration(
    const std::map<std::string, std::string>& config)
{
    m_config = config;
}

bool SBPLPlanningContext::initSBPL(std::string& why)
{
    planning_scene::PlanningSceneConstPtr planning_scene = getPlanningScene();
    if (!planning_scene) {
        why = "No Planning Scene available";
        return false;
    }

    const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();

    const std::string& planning_frame = planning_scene->getPlanningFrame();

    ////////////////////
    // Collision Model
    ////////////////////

    if (!m_collision_checker.init(m_robot_model, planning_scene)) {
        why = "Failed to initialize sbpl Collision Checker "
                "from Planning Scene and Robot Model";
        return false;
    }

    ///////////////////////////////
    // Action Set Initialization
    ///////////////////////////////

    for (size_t vind = 0; vind < m_robot_model->activeVariableCount(); ++vind) {
        std::vector<double> mprim(m_robot_model->activeVariableCount(), 0.0);
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

    moveit_msgs::OrientedBoundingBox workspace_aabb;
    getPlanningFrameWorkspaceAABB(
            req.workspace_parameters, *planning_scene, workspace_aabb);

    ROS_INFO("AABB of workspace in planning frame:");
    ROS_INFO("  pose:");
    ROS_INFO("    position: (%0.3f, %0.3f, %0.3f)", workspace_aabb.pose.position.x, workspace_aabb.pose.position.y, workspace_aabb.pose.position.z);
    ROS_INFO("    orientation: (%0.3f, %0.3f, %0.3f, %0.3f)", workspace_aabb.pose.orientation.w, workspace_aabb.pose.orientation.x, workspace_aabb.pose.orientation.y, workspace_aabb.pose.orientation.z);

    // TODO: block off sections of the aabb that do not include the original
    // workspace

    const double size_x = workspace_aabb.extents.x;
    const double size_y = workspace_aabb.extents.y;
    const double size_z = workspace_aabb.extents.z;

    // TODO: need to either plan in the workspace frame here rather than the
    // common joint root...or need to expand the size of the distance field to
    // match the axis-aligned bb of the workspace and then fill cells outside of
    // the workspace

    const double res_m = 0.02;
    const double max_distance = 0.2;
    const bool propagate_negative_distances = false;

    Eigen::Affine3d T_planning_workspace;
    T_planning_workspace = Eigen::Translation3d(
            workspace_aabb.pose.position.x - 0.5 * workspace_aabb.extents.x,
            workspace_aabb.pose.position.y - 0.5 * workspace_aabb.extents.y,
            workspace_aabb.pose.position.z - 0.5 * workspace_aabb.extents.z);

    Eigen::Vector3d workspace_pos_in_planning(T_planning_workspace.translation());

    ROS_INFO("Initializing workspace distance field:");
    ROS_INFO("  size_x: %0.3f", size_x);
    ROS_INFO("  size_y: %0.3f", size_y);
    ROS_INFO("  size_z: %0.3f", size_z);
    ROS_INFO("  res: %0.3f", res_m);
    ROS_INFO("  origin_x: %0.3f", workspace_pos_in_planning.x());
    ROS_INFO("  origin_y: %0.3f", workspace_pos_in_planning.y());
    ROS_INFO("  origin_z: %0.3f", workspace_pos_in_planning.z());
    ROS_INFO("  propagate_negative_distances: %s", propagate_negative_distances ? "true" : "false");

    m_distance_field.reset(new distance_field::PropagationDistanceField(
            size_x, size_y, size_z,
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
            m_robot_model,
            &m_collision_checker,
            &m_action_set,
            m_distance_field.get()));

    sbpl_arm_planner::PlanningParams params;

    std::vector<int> discretization(m_robot_model->activeVariableCount(), 1);
    std::vector<double> deltas(m_robot_model->activeVariableCount());
    for (size_t vind = 0; vind < deltas.size(); ++vind) {
        deltas[vind] = (double)discretization[vind] / (2.0 * M_PI);
    }
    ROS_INFO("Discretization: %s", leatherman::vectorToString(discretization).c_str());
    ROS_INFO("Deltas: %s", leatherman::vectorToString(deltas).c_str());

    params.num_joints_ = m_robot_model->activeVariableCount();
    params.planning_frame_ = m_robot_model->planningFrame();
    params.group_name_ = m_robot_model->planningGroupName();
    params.planner_name_ = getMotionPlanRequest().planner_id;
    params.planning_joints_ = m_robot_model->planningVariableNames();
    params.coord_vals_ = discretization;
    params.coord_delta_ = deltas;
    params.expands_log_level_ = "debug";
    params.expands2_log_level_ = "debug";

    if (!m_planner->init(params)) {
        why = "Failed to initialize SBPL Arm Planner Interface";
        return false;
    }

    return true;
}

bool SBPLPlanningContext::translateRequest(
    moveit_msgs::MotionPlanRequest& req)
{
    // TODO: translate goal position constraints into planning frame
    // TODO: translate goal orientation constraints into planning frame

    req = getMotionPlanRequest();
    return true;
}

bool SBPLPlanningContext::getPlanningFrameWorkspaceAABB(
    const moveit_msgs::WorkspaceParameters& workspace,
    const planning_scene::PlanningScene& scene,
    moveit_msgs::OrientedBoundingBox& aabb)
{
    if (!scene.knowsFrameTransform(workspace.header.frame_id) ||
        !scene.knowsFrameTransform(scene.getPlanningFrame()))
    {
        return false;
    }

    const Eigen::Affine3d& T_scene_workspace =
            scene.getFrameTransform(workspace.header.frame_id);
    const Eigen::Affine3d& T_scene_planning =
            scene.getFrameTransform(scene.getPlanningFrame());

    Eigen::Affine3d T_planning_workspace =
            T_scene_planning.inverse() * T_scene_workspace;

    Eigen::Vector3d min(
            workspace.min_corner.x,
            workspace.min_corner.y,
            workspace.min_corner.z);
    Eigen::Vector3d max(
            workspace.max_corner.x,
            workspace.max_corner.y,
            workspace.max_corner.z);

    Eigen::Vector3d min_planning = T_planning_workspace * min;
    Eigen::Vector3d max_planning = T_planning_workspace * max;

    const double mid_x = 0.5 * (min_planning.x() + max_planning.x());
    const double mid_y = 0.5 * (min_planning.y() + max_planning.y());
    const double mid_z = 0.5 * (min_planning.z() + max_planning.z());

    double size_x = max_planning.x() - min_planning.x();
    double size_y = max_planning.y() - min_planning.y();
    double size_z = max_planning.z() - min_planning.z();

    aabb.pose.position.x = mid_x;
    aabb.pose.position.y = mid_y;
    aabb.pose.position.z = mid_z;
    aabb.pose.orientation.w = 1.0;
    aabb.extents.x = size_x;
    aabb.extents.y = size_y;
    aabb.extents.z = size_z;

    return true;
}

} // namespace sbpl_interface
