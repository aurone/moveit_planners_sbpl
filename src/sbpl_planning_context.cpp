#include "sbpl_planning_context.h"

// system includes
#include <moveit/collision_detection/world.h>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <sbpl_arm_planner/arm_planner_interface.h>
#include <sbpl_arm_planner/motion_primitive.h>

// project includes
#include <moveit_planners_sbpl/collision_world_sbpl.h>

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
    ROS_DEBUG("Constructed SBPL Planning Context");
}

SBPLPlanningContext::~SBPLPlanningContext()
{
    ROS_DEBUG("Destructed SBPL Planning Context");
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
    planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
    assert(scene);
    moveit::core::RobotModelConstPtr robot = scene->getRobotModel();
    assert(robot);
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

    ROS_DEBUG("Successfully initialized SBPL");

    // translate planning scene to planning scene message
    moveit_msgs::PlanningScenePtr scene_msg(new moveit_msgs::PlanningScene);
    scene->getPlanningSceneMsg(*scene_msg);

    moveit_msgs::MotionPlanRequest req_msg;
    if (!translateRequest(req_msg)) {
        ROS_WARN("Unable to translate Motion Plan Request to SBPL Motion Plan Request");
        return false;
    }

    robot_state::RobotStateConstPtr start_state =
            scene->getCurrentStateUpdated(req_msg.start_state);
    if (!start_state) {
        ROS_WARN("Unable to update start state with requested start state overrides");
        return false;
    }
    moveit::core::robotStateToRobotStateMsg(*start_state, req_msg.start_state);

    moveit_msgs::MotionPlanResponse res_msg;
    bool result = m_planner->solve(scene_msg, req_msg, res_msg);
    if (result) {
        ROS_DEBUG("Call to solve() succeeded");

        ROS_DEBUG("Creating RobotTrajectory from path with %zu joint trajectory points and %zu multi-dof joint trajectory points",
                    res_msg.trajectory.joint_trajectory.points.size(),
                    res_msg.trajectory.multi_dof_joint_trajectory.points.size());
        robot_trajectory::RobotTrajectoryPtr traj(
                new robot_trajectory::RobotTrajectory(robot, getGroupName()));
        traj->setRobotTrajectoryMsg(*start_state, res_msg.trajectory);

        // TODO: Is there any reason to use res_msg.trajectory_start as the
        // reference state or res_msg.group_name in the above RobotTrajectory
        // constructor?

        ROS_INFO("Motion Plan Response:");
        ROS_INFO("  Trajectory: %zu points", traj->getWayPointCount());
        ROS_INFO("  Planning Time: %0.3f seconds", res_msg.planning_time);
        ROS_INFO("  Error Code: %d (%s)", res_msg.error_code.val, to_string(res_msg.error_code).c_str());

        res.trajectory_ = traj;
        res.planning_time_ = res_msg.planning_time;
        res.error_code_ = res_msg.error_code;
    }
    else {
        res.trajectory_.reset();
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
    ROS_DEBUG("Initializing SBPL Planning Context");

    // TODO: implement a way to pass down planner-specific parameters to the
    // SBPL Arm Planner Interface from above and to query the interface for
    // expected parameters

    if (!m_robot_model->initialized()) {
        ROS_ERROR("MoveIt! Robot Model is not initialized");
        return false;
    }

    //////////////////////////
    // check for parameters //
    //////////////////////////

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
        "shortcut_path",
        "interpolate_path"
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
        "bfs_sphere_radius"
    };

    if (config.at("use_bfs_heuristic") == "true") {
        for (const std::string& req_param : bfs_required_params) {
            if (config.find(req_param) == config.end()) {
                ROS_ERROR("Missing parameter '%s'", req_param.c_str());
                return false;
            }
        }
    }

    ROS_DEBUG(" -> Required Parameters Found");

    ///////////////////////////////////
    // parse and validate parameters //
    ///////////////////////////////////

    std::map<std::string, double> disc;

    bool use_xyz_snap_mprim = config.at("use_xyz_snap_mprim") == "true";
    bool use_rpy_snap_mprim = config.at("use_rpy_snap_mprim") == "true";
    bool use_xyzrpy_snap_mprim = config.at("use_xyzrpy_snap_mprim") == "true";
    bool use_short_dist_mprims = config.at("use_short_dist_mprims") == "true";
    double xyz_snap_thresh = 0.0;
    double rpy_snap_thresh = 0.0;
    double xyzrpy_snap_thresh = 0.0;
    double short_dist_mprims_thresh = 0.0;
    try {
        xyz_snap_thresh = std::stod(config.at("xyz_snap_dist_thresh"));
        rpy_snap_thresh = std::stod(config.at("rpy_snap_dist_thresh"));
        xyzrpy_snap_thresh = std::stod(config.at("xyzrpy_snap_dist_thresh"));
        short_dist_mprims_thresh = std::stod(config.at("short_dist_mprims_thresh"));
    }
    catch (const std::logic_error& ex) {
        ROS_ERROR("Failed to convert amp distance thresholds to floating-point values");
        return false;
    }

    double epsilon;
    try {
        epsilon = std::stod(config.at("epsilon"));
    }
    catch (const std::logic_error& ex) {
        ROS_ERROR("Failed to convert epsilon to floating-point value");
        return false;
    }

    bool shortcut_path = config.at("shortcut_path") == "true";
    sbpl::manip::ShortcutType shortcut_type =
            sbpl::manip::PlanningParams::DefaultShortcutType;
    if (shortcut_path) {
        auto it = config.find("shortcut_type");
        if (it != config.end()) {
            if (it->second == "joint_space") {
                shortcut_type = sbpl::manip::ShortcutType::JOINT_SPACE;
            }
            else if (it->second == "euclidean") {
                shortcut_type = sbpl::manip::ShortcutType::EUCLID_SPACE;
            }
            else {
                ROS_WARN("Parameter 'shortcut_path' has unrecognized value");
            }
        }
    }
    bool interpolate_path = config.at("interpolate_path") == "true";

    bool use_bfs_heuristic = config.at("use_bfs_heuristic") == "true";
    double bfs_res_x = 0.0;
    double bfs_res_y = 0.0;
    double bfs_res_z = 0.0;
    double bfs_sphere_radius = 0.0;
    if (use_bfs_heuristic) {
        try {
            bfs_res_x = std::stod(config.at("bfs_res_x"));
            bfs_res_y = std::stod(config.at("bfs_res_y"));
            bfs_res_z = std::stod(config.at("bfs_res_z"));
            bfs_sphere_radius = std::stod(config.at("bfs_sphere_radius"));

            if (bfs_res_x != bfs_res_y || bfs_res_x != bfs_res_z) {
                ROS_WARN("Distance field currently only supports uniformly discretized grids. Using x resolution (%0.3f) as resolution for all dimensions", bfs_res_x);
            }
        }
        catch (const std::logic_error& ex) { // thrown by std::stod
            ROS_ERROR("Failed to convert bfs resolutions to floating-point values");
            return false;
        }
    }

    // check that we have discretization
    std::stringstream ss(config.at("discretization"));
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }

    ROS_DEBUG("Parsed discretization for %zu joints", disc.size());

    // TODO: check that we have discretization for all active planning variables

    const std::string& action_filename = config.at("mprim_filename");

    sbpl::manip::ActionSet* as = new sbpl::manip::ActionSet;

    if (!as) {
        ROS_ERROR("Failed to instantiate appropriate Action Set");
        return false;
    }

    if (!sbpl::manip::ActionSet::Load(action_filename, *as)) {
        ROS_ERROR("Failed to load action set from '%s'", action_filename.c_str());
        delete as;
        return false;
    }

    //////////////////////////////////////////////
    // initialize structures against parameters //
    //////////////////////////////////////////////

    m_config = config;

    m_disc = disc;
    m_action_set.reset(as);
    m_use_xyz_snap_mprim = use_xyz_snap_mprim;
    m_use_rpy_snap_mprim = use_rpy_snap_mprim;
    m_use_xyzrpy_snap_mprim = use_xyzrpy_snap_mprim;
    m_use_short_dist_mprims = use_short_dist_mprims;
    m_xyz_snap_thresh = xyz_snap_thresh;
    m_rpy_snap_thresh = rpy_snap_thresh;
    m_xyzrpy_snap_thresh = xyzrpy_snap_thresh;
    m_short_dist_mprims_thresh = short_dist_mprims_thresh;

    m_use_bfs_heuristic = use_bfs_heuristic;
    m_bfs_res_x = bfs_res_x;
    m_bfs_res_y = bfs_res_y;
    m_bfs_res_z = bfs_res_z;
    m_bfs_sphere_radius = bfs_sphere_radius;

    m_epsilon = epsilon;

    m_shortcut_path = shortcut_path;
    m_shortcut_type = shortcut_type;
    m_interpolate_path = interpolate_path;

    m_action_set->useAmp(
            sbpl::manip::MotionPrimitive::SNAP_TO_XYZ,
            use_xyz_snap_mprim);
    m_action_set->useAmp(
            sbpl::manip::MotionPrimitive::SNAP_TO_RPY,
            use_rpy_snap_mprim);
    m_action_set->useAmp(
            sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY,
            use_xyzrpy_snap_mprim);
    m_action_set->useAmp(
            sbpl::manip::MotionPrimitive::SHORT_DISTANCE,
            use_short_dist_mprims);

    m_action_set->ampThresh(
            sbpl::manip::MotionPrimitive::SNAP_TO_XYZ,
            xyz_snap_thresh);
    m_action_set->ampThresh(
            sbpl::manip::MotionPrimitive::SNAP_TO_RPY,
            rpy_snap_thresh);
    m_action_set->ampThresh(
            sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY,
            xyzrpy_snap_thresh);
    m_action_set->ampThresh(
            sbpl::manip::MotionPrimitive::SHORT_DISTANCE,
            short_dist_mprims_thresh);

    ROS_DEBUG("Action Set:");
    for (auto ait = m_action_set->begin(); ait != m_action_set->end(); ++ait) {
        ROS_DEBUG("  type: %s", to_string(ait->type).c_str());
        if (ait->type == sbpl::manip::MotionPrimitive::SNAP_TO_RPY) {
            ROS_DEBUG("    enabled: %s", m_action_set->useAmp(sbpl::manip::MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            ROS_DEBUG("    thresh: %0.3f", m_action_set->ampThresh(sbpl::manip::MotionPrimitive::SNAP_TO_RPY));
        }
        else if (ait->type == sbpl::manip::MotionPrimitive::SNAP_TO_XYZ) {
            ROS_DEBUG("    enabled: %s", m_action_set->useAmp(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            ROS_DEBUG("    thresh: %0.3f", m_action_set->ampThresh(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ));
        }
        else if (ait->type == sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_DEBUG("    enabled: %s", m_action_set->useAmp(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            ROS_DEBUG("    thresh: %0.3f", m_action_set->ampThresh(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY));
        }
        else if (ait->type == sbpl::manip::MotionPrimitive::LONG_DISTANCE ||
            ait->type == sbpl::manip::MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_DEBUG("    action: %s", to_string(ait->action).c_str());
        }
    }

    return true;
}

bool SBPLPlanningContext::initSBPL(std::string& why)
{
    planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
    if (!scene) {
        why = "No Planning Scene available";
        return false;
    }

    const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();

    const std::string& planning_frame = scene->getPlanningFrame();

    ////////////////////
    // Collision Model
    ////////////////////

    moveit::core::RobotModelConstPtr robot = scene->getRobotModel();
    moveit::core::RobotState start_state(robot);

    if (!moveit::core::robotStateMsgToRobotState(req.start_state, start_state)) {
        why = "Failed to convert start state to reference state";
        return false;
    }

    if (!m_collision_checker.init(m_robot_model, start_state, scene)) {
        why = "Failed to initialize sbpl Collision Checker "
                "from Planning Scene and Robot Model";
        return false;
    }

    ///////////////////////////////
    // Action Set Initialization //
    ///////////////////////////////

    // TODO: the motion primitive file currently must have joint variable
    // changes specified in the active variable order...which the user may not
    // know...fix that

    // Action Set loaded upon initialization

    //////////////////////////////////
    // Distance Field Initalization //
    //////////////////////////////////

    if (!initHeuristicGrid(*scene, req.workspace_parameters)) {
        why = "Failed to initialize heuristic information";
        return false;
    }

    ///////////////////////////////////////////////
    // SBPL Arm Planner Interface Initialization //
    ///////////////////////////////////////////////

    m_planner.reset(new sbpl::manip::ArmPlannerInterface(
            m_robot_model,
            &m_collision_checker,
            m_action_set.get(),
            m_grid.get()));

    sbpl::manip::PlanningParams params;

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

    params.planning_frame_ = m_robot_model->planningFrame();
    params.num_joints_ = m_robot_model->activeVariableCount();
    params.planning_joints_ = m_robot_model->planningVariableNames();
    params.coord_vals_ = discretization;
    params.coord_delta_ = deltas;

    params.use_multiple_ik_solutions_ = false;

    // NOTE: default cost function parameters

    params.use_bfs_heuristic_ = m_use_bfs_heuristic;
    params.planning_link_sphere_radius_ = m_bfs_sphere_radius;

    params.planner_name_ = getMotionPlanRequest().planner_id;
    params.epsilon_ = m_epsilon;

    params.print_path_ = false;
    params.shortcut_path_ = m_shortcut_path;
    params.shortcut_type = m_shortcut_type;
    params.interpolate_path_ = m_interpolate_path;

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

bool SBPLPlanningContext::initHeuristicGrid(
    const planning_scene::PlanningScene& scene,
    const moveit_msgs::WorkspaceParameters& workspace)
{
    // TODO: find the transform from the workspace to the planning frame and
    // use as the origin

    // create a distance field in the planning frame that represents the
    // workspace boundaries

    const auto& req = getMotionPlanRequest();

    /////////////////////////////////////////
    // Determine Distance Field Parameters //
    /////////////////////////////////////////

    moveit_msgs::OrientedBoundingBox workspace_aabb;
    if (!getPlanningFrameWorkspaceAABB(
            req.workspace_parameters, scene, workspace_aabb))
    {
        ROS_ERROR("Failed to get workspace boundaries in the planning frame");
        return false;
    }

    ROS_DEBUG("AABB of workspace in planning frame:");
    ROS_DEBUG("  pose:");
    ROS_DEBUG("    position: (%0.3f, %0.3f, %0.3f)", workspace_aabb.pose.position.x, workspace_aabb.pose.position.y, workspace_aabb.pose.position.z);
    ROS_DEBUG("    orientation: (%0.3f, %0.3f, %0.3f, %0.3f)", workspace_aabb.pose.orientation.w, workspace_aabb.pose.orientation.x, workspace_aabb.pose.orientation.y, workspace_aabb.pose.orientation.z);

    // TODO: block off sections of the aabb that do not include the original
    // workspace

    const double size_x = workspace_aabb.extents.x;
    const double size_y = workspace_aabb.extents.y;
    const double size_z = workspace_aabb.extents.z;

    // TODO: need to either plan in the workspace frame here rather than the
    // common joint root...or need to expand the size of the distance field to
    // match the axis-aligned bb of the workspace and then fill cells outside of
    // the workspace

    const double default_bfs_res = 0.02;
    const double res_x_m = m_use_bfs_heuristic ? m_bfs_res_x : default_bfs_res;
    const double res_y_m = m_use_bfs_heuristic ? m_bfs_res_y : default_bfs_res;
    const double res_z_m = m_use_bfs_heuristic ? m_bfs_res_z : default_bfs_res;

    const double max_distance = m_use_bfs_heuristic ? m_bfs_sphere_radius + res_x_m : res_x_m;
    const bool propagate_negative_distances = false;

    Eigen::Affine3d T_planning_workspace;
    T_planning_workspace = Eigen::Translation3d(
            workspace_aabb.pose.position.x - 0.5 * workspace_aabb.extents.x,
            workspace_aabb.pose.position.y - 0.5 * workspace_aabb.extents.y,
            workspace_aabb.pose.position.z - 0.5 * workspace_aabb.extents.z);

    Eigen::Vector3d workspace_pos_in_planning(T_planning_workspace.translation());

    ROS_DEBUG("Initializing workspace distance field:");
    ROS_DEBUG("  size_x: %0.3f", size_x);
    ROS_DEBUG("  size_y: %0.3f", size_y);
    ROS_DEBUG("  size_z: %0.3f", size_z);
    ROS_DEBUG("  res: %0.3f", res_x_m);
    ROS_DEBUG("  origin_x: %0.3f", workspace_pos_in_planning.x());
    ROS_DEBUG("  origin_y: %0.3f", workspace_pos_in_planning.y());
    ROS_DEBUG("  origin_z: %0.3f", workspace_pos_in_planning.z());
    ROS_DEBUG("  propagate_negative_distances: %s", propagate_negative_distances ? "true" : "false");

    m_distance_field = std::make_shared<distance_field::PropagationDistanceField>(
            size_x, size_y, size_z,
            res_x_m,
            workspace_pos_in_planning.x(),
            workspace_pos_in_planning.y(),
            workspace_pos_in_planning.z(),
            max_distance,
            propagate_negative_distances);

    if (!m_use_bfs_heuristic) {
        ROS_DEBUG("Not using BFS heuristic (Skipping occupancy grid filling)");
        return true;
    }

    /////////////////////////
    // Fill Distance Field //
    /////////////////////////

    collision_detection::CollisionWorldConstPtr cworld =
            scene.getCollisionWorld();
    const collision_detection::CollisionWorldSBPL* sbpl_cworld =
            dynamic_cast<const collision_detection::CollisionWorldSBPL*>(
                    cworld.get());
    if (sbpl_cworld) {
        ROS_DEBUG("Using collision information from Collision World SBPL for heuristic!!!");

        const distance_field::PropagationDistanceField* df =
                sbpl_cworld->distanceField(m_robot_model->planningGroupName());
        if (!df) {
            ROS_WARN("Just kidding! Collision World SBPL's distance field is uninitialized");
            return true;
        }

        // copy the collision information
        ROS_DEBUG("Copying collision information");

        EigenSTL::vector_Vector3d points;
        for (int x = 0; x < m_distance_field->getXNumCells(); ++x) {
            for (int y = 0; y < m_distance_field->getYNumCells(); ++y) {
                for (int z = 0; z < m_distance_field->getZNumCells(); ++z) {
                    double wx, wy, wz;
                    m_distance_field->gridToWorld(x, y, z, wx, wy, wz);
                    if (df->getDistance(wx, wy, wz) <= 0.0) {
                        // convert x, y, z to world space
                        // transform back into the world frame
                        points.emplace_back(wx, wy, wz);
                    }
                }
            }
        }

        ROS_DEBUG("Adding %zu points to the bfs distance field", points.size());
        m_distance_field->addPointsToField(points);
        return true;
    }
    else {
        m_grid = std::make_shared<sbpl::OccupancyGrid>(m_distance_field);
        m_grid->setReferenceFrame(scene.getPlanningFrame());
        sbpl::collision::WorldCollisionModel cmodel(m_grid.get());

        // insert world objects into the collision model
        collision_detection::WorldConstPtr world = cworld->getWorld();
        if (world) {
            int insert_count = 0;
            for (auto oit = world->begin(); oit != world->end(); ++oit) {
                if (!cmodel.insertObject(oit->second)) {
                    ROS_WARN("Failed to insert object '%s' into heuristic grid", oit->first.c_str());
                }
                else {
                    ++insert_count;
                }
            }
            ROS_DEBUG("Inserted %d objects into the heuristic grid", insert_count);
        }
        else {
            ROS_WARN("Attempt to insert null World into heuristic grid");
        }

        // note: collision world and going out of scope here will
        // not destroy the prepared distance field and occupancy grid
    }

    return true;
}

} // namespace sbpl_interface
