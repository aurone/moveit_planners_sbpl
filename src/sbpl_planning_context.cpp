#include "sbpl_planning_context.h"

#include <leatherman/print.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <sbpl_manipulation_components/motion_primitive.h>

namespace moveit_msgs {

static std::string to_string(moveit_msgs::MoveItErrorCodes code)
{
    switch (code.val) {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::FAILURE:
        return "FAILURE";

    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
        return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
        return "PREEMPTED";

    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_PATH_CONSTRAINTS";

    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";

    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
        return "INVALID_LINK_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
        return "INVALID_OBJECT_NAME";

    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
        return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
        return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
        return "ROBOT_STATE_STALE";
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
        return "SENSOR_INFO_STALE";

    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";

    default:
        return "UNRECOGNIZED";
    }
}

} // namespace moveit_msgs

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

    planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
    moveit::core::RobotModelConstPtr robot = scene->getRobotModel();
    const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();

    if (req.goal_constraints.empty()) {
        // :3
        res.trajectory_.reset(
                new robot_trajectory::RobotTrajectory(robot, getGroupName()));
        res.planning_time_ = 0.0;
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    std::string why;
    if (!initSBPL(why)) {
        ROS_WARN("Failed to initialize SBPL (%s)", why.c_str());
        res.planning_time_ = 0.0;
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    ROS_INFO("Successfully initialized SBPL");

    moveit_msgs::PlanningScenePtr scene_msg(new moveit_msgs::PlanningScene);

    assert(scene.get()); // otherwise initialization would have failed

    scene->getPlanningSceneMsg(*scene_msg);

    moveit_msgs::MotionPlanRequest req_msg;
    if (!translateRequest(req_msg)) {
        ROS_WARN("Unable to translate Motion Plan Request to SBPL Motion Plan Request");
        return false;
    }

    moveit_msgs::MotionPlanResponse res_msg;
    bool result = m_planner->solve(scene_msg, req_msg, res_msg);
    if (result) {
        ROS_INFO("Call to SBPLArmPlannerInterface::solve() succeeded");

        moveit::core::RobotState ref_state(robot);
        robot_state::RobotStatePtr start_state =
                scene->getCurrentStateUpdated(req.start_state);

        ROS_INFO("Creating RobotTrajectory from path with %zu joint trajectory points and %zu multi-dof joint trajectory points",
                    res_msg.trajectory.joint_trajectory.points.size(),
                    res_msg.trajectory.multi_dof_joint_trajectory.points.size());
        robot_trajectory::RobotTrajectoryPtr traj(
                new robot_trajectory::RobotTrajectory(
                        robot, getGroupName()));
        traj->setRobotTrajectoryMsg(
                *start_state, res_msg.trajectory);

        // res_msg
        //     trajectory_start
        //     group_name
        //     trajectory
        //     planning_time
        //     error_code

        ROS_INFO("Motion Plan Response:");
        ROS_INFO("  Trajectory: %zu points", traj->getWayPointCount());
        ROS_INFO("  Planning Time: %0.3f seconds", res_msg.planning_time);
        ROS_INFO("  Error Code: %d (%s)", res_msg.error_code.val, to_string(res_msg.error_code).c_str());

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

bool SBPLPlanningContext::init(const std::map<std::string, std::string>& config)
{
    ROS_INFO("Initializing SBPL Planning Context");

    // check for required parameters
    const std::vector<std::string>& required_params =
    {
        // environment
        "discretization",

        // action set
        "mprim_filename",
        "use_rpy_snap_mprim",
        "use_xyz_snap_mprim",
        "use_xyzrpy_snap_mprim",
        "use_short_dist_mprims",
        "xyz_snap_dist_thresh",
        "rpy_snap_dist_thresh",
        "xyzrpy_snap_dist_thresh",
        "short_dist_mprims_thresh",

        "use_bfs_heuristic",

        // planner
        "type",
        "epsilon",

        // post-processing
        "shortcut_path"
    };

    for (const std::string& req_param : required_params) {
        if (config.find(req_param) == config.end()) {
            ROS_ERROR("Missing parameter '%s'", req_param.c_str());
            return false;
        }
    }

    // check for conditionally-required parameters
    const std::vector<std::string>& bfs_required_params =
    {
        "bfs_res_x",
        "bfs_res_y",
        "bfs_res_z",
    };
    if (config.at("use_bfs_heuristic") == "true") {
        for (const std::string& req_param : bfs_required_params) {
            if (config.find(req_param) == config.end()) {
                ROS_ERROR("Missing parameter '%s'", req_param.c_str());
                return false;
            }
        }
    }

    ROS_INFO(" -> Required Parameters Found");

    // parse parameters and check validity

    std::map<std::string, double> disc;

    bool use_xyz_snap_mprim = config.at("use_xyz_snap_mprim") == "true";
    bool use_rpy_snap_mprim = config.at("use_rpy_snap_mprim") == "true";
    bool use_xyzrpy_snap_mprim = config.at("use_xyzrpy_snap_mprim") == "true";
    bool use_short_dist_mprims = config.at("use_short_dist_mprims") == "true";
    double xyz_snap_thresh = config.at("xyz_snap_dist_thresh") == "true";
    double rpy_snap_thresh = config.at("rpy_snap_dist_thresh") == "true";
    double xyzrpy_snap_thresh = config.at("xyzrpy_snap_dist_thresh") == "true";
    double short_dist_mprims_thresh =
            config.at("short_dist_mprims_thresh") == "true";

    bool shortcut_path = config.at("shortcut_path") == "true";

    // check that we have discretization
    std::stringstream ss(config.at("discretization"));
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }

    ROS_INFO("Parsed discretization for %zu joints", disc.size());

    // TODO: check that we have discretization for all active planning variables

    const std::string& action_filename = config.at("mprim_filename");

    sbpl_arm_planner::ActionSet as;
    if (!sbpl_arm_planner::ActionSet::Load(action_filename, as)) {
        ROS_ERROR("Failed to load action set from '%s'", action_filename.c_str());
        return false;
    }

    m_config = config;

    m_disc = disc;
    m_action_set = as;
    m_use_xyz_snap_mprim = use_xyz_snap_mprim;
    m_use_rpy_snap_mprim = use_rpy_snap_mprim;
    m_use_xyzrpy_snap_mprim = use_xyzrpy_snap_mprim;
    m_use_short_dist_mprims = use_short_dist_mprims;
    m_xyz_snap_thresh = xyz_snap_thresh;
    m_rpy_snap_thresh = rpy_snap_thresh;
    m_xyzrpy_snap_thresh = xyzrpy_snap_thresh;
    m_short_dist_mprims_thresh = short_dist_mprims_thresh;
    m_shortcut_path = shortcut_path;

    m_action_set.useAmp(
            sbpl_arm_planner::MotionPrimitive::SNAP_TO_XYZ,
            use_xyz_snap_mprim);
    m_action_set.useAmp(
            sbpl_arm_planner::MotionPrimitive::SNAP_TO_RPY,
            use_rpy_snap_mprim);
    m_action_set.useAmp(
            sbpl_arm_planner::MotionPrimitive::SNAP_TO_XYZ_RPY,
            use_xyzrpy_snap_mprim);
    m_action_set.useAmp(
            sbpl_arm_planner::MotionPrimitive::SHORT_DISTANCE,
            use_short_dist_mprims);

    m_action_set.ampThresh(
            sbpl_arm_planner::MotionPrimitive::SNAP_TO_XYZ,
            xyz_snap_thresh);
    m_action_set.ampThresh(
            sbpl_arm_planner::MotionPrimitive::SNAP_TO_RPY,
            rpy_snap_thresh);
    m_action_set.ampThresh(
            sbpl_arm_planner::MotionPrimitive::SNAP_TO_XYZ_RPY,
            xyzrpy_snap_thresh);
    m_action_set.ampThresh(
            sbpl_arm_planner::MotionPrimitive::SHORT_DISTANCE,
            short_dist_mprims_thresh);

    ROS_INFO("Action Set:");
    for (auto ait = m_action_set.begin(); ait != m_action_set.end(); ++ait) {
        ROS_INFO("  type: %s", to_string(ait->type).c_str());
        if (ait->type == sbpl_arm_planner::MotionPrimitive::LONG_DISTANCE ||
            ait->type == sbpl_arm_planner::MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_INFO("    action: %s", to_string(ait->action).c_str());
        }
    }

    return true;
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

    // TODO: the motion primitive file currently must have joint variable
    // changes specified in the active variable order...which the user may not
    // know...fix that

    // Action Set loaded upon initialization

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

    // number of discretizations in a circle
    std::vector<int> discretization(m_robot_model->activeVariableCount());
    for (size_t vind = 0; vind < discretization.size(); ++vind) {
        const std::string& vname = m_robot_model->planningVariableNames()[vind];
        auto dit = m_disc.find(vname);
        if (dit == m_disc.end()) {
            ROS_ERROR("Discretization for variable '%s' not available in config", vname.c_str());
            return false;
        }
        discretization[vind] = (int)round((2.0 * M_PI) / dit->second);
    }

    std::vector<double> deltas(m_robot_model->activeVariableCount());
    for (size_t vind = 0; vind < deltas.size(); ++vind) {
        deltas[vind] = (2.0 * M_PI) / (double)discretization[vind];
    }

    ROS_INFO("Discretization: %s", to_string(discretization).c_str());
    ROS_INFO("Deltas: %s", to_string(deltas).c_str());

    params.num_joints_ = m_robot_model->activeVariableCount();
    params.planning_frame_ = m_robot_model->planningFrame();
    params.group_name_ = m_robot_model->planningGroupName();
    params.planner_name_ = getMotionPlanRequest().planner_id;
    params.planning_joints_ = m_robot_model->planningVariableNames();
    params.coord_vals_ = discretization;
    params.coord_delta_ = deltas;
    params.expands_log_level_ = "debug";
    params.expands2_log_level_ = "debug";
    params.shortcut_path_ = m_shortcut_path;

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
