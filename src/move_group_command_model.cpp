#include "move_group_command_model.h"

// standard includes
#include <assert.h>
#include <chrono>
#include <stack>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/utils.h>

namespace sbpl_interface {

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

MoveGroupCommandModel::MoveGroupCommandModel(QObject* parent) :
    QObject(parent),
    m_nh(),
    m_scene_monitor(),
    m_robot_state(),
    m_validity(boost::indeterminate),
    m_check_state_validity_client(),
    m_query_planner_interface_client(),
    m_move_group_client(),
    m_planner_interfaces(),
    m_curr_planner_idx(-1),
    m_curr_planner_id_idx(-1),
    m_available_frames(),
    m_joint_tol_rad(sbpl::utils::ToRadians(DefaultGoalJointTolerance_deg)),
    m_pos_tol_m(DefaultGoalPositionTolerance_m),
    m_rot_tol_rad(sbpl::utils::ToRadians(DefaultGoalOrientationTolerance_deg)),
    m_workspace(),
    m_num_planning_attempts(DefaultNumPlanningAttempts),
    m_allowed_planning_time_s(DefaultAllowedPlanningTime_s),
    m_curr_joint_group_name(),
    m_im_server("phantom_controls"),
    m_int_marker_names()
{
    m_workspace.min_corner.x = DefaultWorkspaceMinX;
    m_workspace.min_corner.y = DefaultWorkspaceMinY;
    m_workspace.min_corner.z = DefaultWorkspaceMinZ;
    m_workspace.max_corner.x = DefaultWorkspaceMaxX;
    m_workspace.max_corner.y = DefaultWorkspaceMaxY;
    m_workspace.max_corner.z = DefaultWorkspaceMaxZ;

    reinitCheckStateValidityService();
    reinitQueryPlannerInterfaceService();

    if (m_query_planner_interface_client->exists()) {
        moveit_msgs::QueryPlannerInterfaces::Request req;
        moveit_msgs::QueryPlannerInterfaces::Response res;
        if (!m_query_planner_interface_client->call(req, res)) {
            ROS_ERROR("Failed to call service '%s'", m_query_planner_interface_client->getService().c_str());
        }
        else {
            m_planner_interfaces = res.planner_interfaces;
            if (!m_planner_interfaces.empty()) {
                m_curr_planner_idx = 0;
                if (!m_planner_interfaces[0].planner_ids.empty()) {
                    m_curr_planner_id_idx = 0;
                }
            }
        }
    }

    m_move_group_client.reset(new MoveGroupActionClient("move_group", false));
}

MoveGroupCommandModel::~MoveGroupCommandModel()
{
    if (m_scene_monitor) {
        m_scene_monitor->stopSceneMonitor();
        m_scene_monitor->stopStateMonitor();
    }
}

bool MoveGroupCommandModel::loadRobot(const std::string& robot_description)
{
    if (robot_description == robotDescription()) {
        return true;
    }

    auto transformer = boost::shared_ptr<tf::Transformer>(new tf::TransformListener);
    m_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(
            robot_description, transformer));
    if (!m_scene_monitor) {
        ROS_ERROR("Failed to instantiate Planning Scene Monitor");
        return false;
    }
    if (!m_scene_monitor->getRobotModel()) {
        ROS_ERROR("Planning Scene Monitor failed to parse robot description");
        return false;
        m_scene_monitor.reset();
    }

    ROS_INFO("Created new Planning Scene Monitor");

    m_scene_monitor->requestPlanningSceneState();
    auto update_fn = boost::bind(
            &MoveGroupCommandModel::processSceneUpdate, this, _1);
    m_scene_monitor->addUpdateCallback(update_fn);
    m_scene_monitor->startSceneMonitor();
    m_scene_monitor->startStateMonitor();

    logPlanningSceneMonitor(*m_scene_monitor);

    m_robot_state.reset(new moveit::core::RobotState(robotModel()));

    m_robot_state->setToDefaultValues();
    m_robot_state->updateLinkTransforms();
    m_robot_state->updateCollisionBodyTransforms();

    if (!robotModel()->hasJointModelGroup(m_curr_joint_group_name)) {
        if (!robotModel()->getJointModelGroupNames().empty()) {
            m_curr_joint_group_name =
                    robotModel()->getJointModelGroupNames().front();
        }
    }
    // otherwise retain the selected joint group

    logRobotModelInfo(*robotModel());

    reinitInteractiveMarkers();

    // monitor the current planning scene for the current robot state

    clearMoveGroupRequest();

    Q_EMIT robotLoaded();

    // seed the available transforms using the available links
    for (const auto& link_name : robotModel()->getLinkModelNames()) {
        m_available_frames.push_back(link_name);
    }
    // NOTE: this emission has to come after the robotLoaded() signal has been
    // emitted above...not sure why this is yet but failure to do so results in
    // a repeatable crash
    Q_EMIT availableFramesUpdated();

    return true;
}

bool MoveGroupCommandModel::isRobotLoaded() const
{
    return (bool)m_scene_monitor.get();
}

moveit::core::RobotModelConstPtr MoveGroupCommandModel::robotModel() const
{
    if (m_scene_monitor) {
        return m_scene_monitor->getRobotModel();
    }
    else {
        return moveit::core::RobotModelConstPtr();
    }
}

moveit::core::RobotStateConstPtr MoveGroupCommandModel::robotState() const
{
    return m_robot_state;
}

bool MoveGroupCommandModel::readyToPlan() const
{
    if (!isRobotLoaded()) {
        ROS_WARN("Cannot plan with no robot loaded");
        return false;
    }

    if (plannerName() == "UNKNOWN" || plannerID() == "UNKNOWN") {
        ROS_WARN("Cannot plan without planner specified");
        return false;
    }

    return true;
}

bool MoveGroupCommandModel::planToGoalPose(const std::string& group_name)
{
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = true;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = true;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    return sendMoveGroupPoseGoal(group_name, ops);
}

bool MoveGroupCommandModel::planToGoalConfiguration(
    const std::string& group_name)
{
    ROS_ERROR("planToGoalConfiguration unimplemented");
    return false;
}

bool MoveGroupCommandModel::moveToGoalPose(const std::string& group_name)
{
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = true;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = false;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    return sendMoveGroupPoseGoal(group_name, ops);
}

bool MoveGroupCommandModel::moveToGoalConfiguration(const std::string& group_name)
{
    ROS_ERROR("moveToGoalConfiguration unimplemented");
    return false;
}

bool MoveGroupCommandModel::copyCurrentState()
{
    if (getActualState(*m_robot_state)) {
        updateInteractiveMarkers();
        Q_EMIT robotStateChanged();
        return true;
    }
    else {
        return false;
    }
}

const std::vector<moveit_msgs::PlannerInterfaceDescription>&
MoveGroupCommandModel::plannerInterfaces() const
{
    return m_planner_interfaces;
}


const std::vector<std::string>&
MoveGroupCommandModel::availableFrames() const
{
    return m_available_frames;
}

const std::string MoveGroupCommandModel::robotDescription() const
{
    if (m_scene_monitor) {
        return m_scene_monitor->getRobotDescription();
    }
    else {
        return std::string();
    }
}

const std::string MoveGroupCommandModel::plannerName() const
{
    assert(m_curr_planner_idx != -1 ?
                m_curr_planner_idx >= 0 &&
                m_curr_planner_idx < (int)m_planner_interfaces.size()
            :
                true);
    if (m_curr_planner_idx == -1) {
        return "UNKNOWN";
    }
    else {
        return m_planner_interfaces[m_curr_planner_idx].name;
    }
}

const std::string MoveGroupCommandModel::plannerID() const
{
    assert(m_curr_planner_id_idx != -1 ?
            m_curr_planner_idx != -1 &&
            m_curr_planner_idx >= 0 &&
            m_curr_planner_idx < (int)m_planner_interfaces.size() &&
            m_curr_planner_id_idx >= 0 &&
            m_curr_planner_id_idx < (int)m_planner_interfaces[m_curr_planner_idx].planner_ids.size()
            :
            true);
    if (m_curr_planner_id_idx == -1) {
        return "UNKNOWN";
    }
    else {
        return m_planner_interfaces[m_curr_planner_idx]
                .planner_ids[m_curr_planner_id_idx];
    }
}

int MoveGroupCommandModel::numPlanningAttempts() const
{
    return m_num_planning_attempts;
}

double MoveGroupCommandModel::allowedPlanningTime() const
{
    return m_allowed_planning_time_s;
}

const std::string& MoveGroupCommandModel::planningJointGroupName() const
{
    return m_curr_joint_group_name;
}

double MoveGroupCommandModel::goalJointTolerance() const
{
    return sbpl::utils::ToDegrees(m_joint_tol_rad);
}

double MoveGroupCommandModel::goalPositionTolerance() const
{
    return m_pos_tol_m;
}

double MoveGroupCommandModel::goalOrientationTolerance() const
{
    return sbpl::utils::ToDegrees(m_rot_tol_rad);
}

const moveit_msgs::WorkspaceParameters& MoveGroupCommandModel::workspace() const
{
    return m_workspace;
}

void MoveGroupCommandModel::load(const rviz::Config& config)
{
    // parse general/robot settings
    QString robot_description;
    config.mapGetString("robot_description", &robot_description);

    // parse planner settings
    int curr_planner_idx = -1;
    int curr_planner_id_idx = -1;
    int num_planning_attempts = DefaultNumPlanningAttempts;
    float allowed_planning_time = DefaultAllowedPlanningTime_s;
    config.mapGetInt("current_planner_index", &curr_planner_idx);
    config.mapGetInt("current_planner_id_index", &curr_planner_id_idx);
    config.mapGetInt("num_planning_attempts", &num_planning_attempts);
    config.mapGetFloat("allowed_planning_time", &allowed_planning_time);

    // parse goal request settings
    QString active_joint_group_name;
    std::vector<std::pair<std::string, double>> joint_variables;
    float joint_tol_rad = sbpl::utils::ToRadians(DefaultGoalJointTolerance_deg);
    float pos_tol_m = DefaultGoalPositionTolerance_m;
    float rot_tol_rad = sbpl::utils::ToRadians(DefaultGoalOrientationTolerance_deg);
    QString ws_frame;
    float ws_min_x = DefaultWorkspaceMinX;
    float ws_min_y = DefaultWorkspaceMinY;
    float ws_min_z = DefaultWorkspaceMinZ;
    float ws_max_x = DefaultWorkspaceMaxX;
    float ws_max_y = DefaultWorkspaceMaxY;
    float ws_max_z = DefaultWorkspaceMaxZ;
    config.mapGetString("active_joint_group", &active_joint_group_name);
    rviz::Config phantom_state_config = config.mapGetChild("phantom_state");
    for (auto it = phantom_state_config.mapIterator();
        it.isValid(); it.advance())
    {
        joint_variables.push_back(std::make_pair(
                it.currentKey().toStdString(),
                it.currentChild().getValue().toDouble()));
    }
    config.mapGetFloat("joint_tolerance", &joint_tol_rad);
    config.mapGetFloat("position_tolerance", &pos_tol_m);
    config.mapGetFloat("orientation_tolerance", &rot_tol_rad);
    config.mapGetString("workspace_frame", &ws_frame);
    config.mapGetFloat("workspace_min_x", &ws_min_x);
    config.mapGetFloat("workspace_min_y", &ws_min_y);
    config.mapGetFloat("workspace_min_z", &ws_min_z);
    config.mapGetFloat("workspace_max_x", &ws_max_x);
    config.mapGetFloat("workspace_max_y", &ws_max_y);
    config.mapGetFloat("workspace_max_z", &ws_max_z);

    ROS_INFO("Loaded Model Configuration:");
    ROS_INFO("  Robot Description: %s", robot_description.toStdString().c_str());
    ROS_INFO("  Current Planner Index: %d", curr_planner_idx);
    ROS_INFO("  Current Planner ID Index: %d", curr_planner_id_idx);
    ROS_INFO("  Num Planning Attempts: %d", num_planning_attempts);
    ROS_INFO("  Allowed Planning Time: %0.3f", allowed_planning_time);
    ROS_INFO("  Active Joint Group: %s", active_joint_group_name.toStdString().c_str());
    ROS_INFO("  Phantom State:");
    for (const auto& entry : joint_variables) {
        ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
    }
    ROS_INFO("  Joint Tolerance (deg): %0.3f", sbpl::utils::ToDegrees(joint_tol_rad));
    ROS_INFO("  Position Tolerance (m): %0.3f", pos_tol_m);
    ROS_INFO("  Orientation Tolerance (deg): %0.3f", rot_tol_rad);

    // set up the model using public member functions to fire off appropriate
    // signals

    const bool robot_loaded = loadRobot(robot_description.toStdString());
    if (!robot_loaded) {
        ROS_WARN("Failed to load robot from recalled parameter name");
    }

    // setup planner settings
    if (plannerIndicesValid(curr_planner_idx, curr_planner_id_idx)) {
        const auto& planner_ifaces = plannerInterfaces();
        const auto& planner_iface = planner_ifaces[curr_planner_idx];
        setPlannerName(planner_iface.name);
        setPlannerID(planner_iface.planner_ids[curr_planner_id_idx]);
    }

    setNumPlanningAttempts(num_planning_attempts);
    setAllowedPlanningTime(allowed_planning_time);

    // setup goal request settings
    if (robot_loaded) {
        setPlanningJointGroup(active_joint_group_name.toStdString());
        for (const auto& entry : joint_variables) {
            const std::string& jv_name = entry.first;
            const double jv_pos = entry.second;
            if (hasVariable(*robotModel(), jv_name)) {
                setJointVariable(jv_name, jv_pos);
            }
        }
    }
    setGoalJointTolerance(sbpl::utils::ToDegrees(joint_tol_rad));
    setGoalPositionTolerance(pos_tol_m);
    setGoalOrientationTolerance(sbpl::utils::ToDegrees(rot_tol_rad));

    moveit_msgs::WorkspaceParameters ws;
    auto it = std::find(
            m_available_frames.begin(), m_available_frames.begin(),
            ws_frame.toStdString());
    if (it != m_available_frames.end()) {
        ws.header.frame_id = ws_frame.toStdString();
    }
    else {
        ws.header.frame_id = "";
    }
    ws.min_corner.x = ws_min_x;
    ws.min_corner.y = ws_min_y;
    ws.min_corner.z = ws_min_z;
    ws.max_corner.x = ws_max_x;
    ws.max_corner.y = ws_max_y;
    ws.max_corner.z = ws_max_z;
    setWorkspace(ws);
}

void MoveGroupCommandModel::save(rviz::Config config) const
{
    ROS_INFO("Saving model configuration");

    // general/robot settings
    config.mapSetValue(
            "robot_description",
            QString::fromStdString(robotDescription()));

    // planner settings
    config.mapSetValue("current_planner_index", m_curr_planner_idx);
    config.mapSetValue("current_planner_id_index", m_curr_planner_id_idx);
    config.mapSetValue("num_planning_attempts", m_num_planning_attempts);
    config.mapSetValue("allowed_planning_time", m_allowed_planning_time_s);

    // goal request settings
    config.mapSetValue("active_joint_group", QString::fromStdString(m_curr_joint_group_name));
    rviz::Config phantom_state_config = config.mapMakeChild("phantom_state");
    if (m_robot_state) {
        for (int vidx = 0; vidx < m_robot_state->getVariableCount(); ++vidx) {
            const std::string& jv_name =
                    m_robot_state->getVariableNames()[vidx];
            phantom_state_config.mapSetValue(
                    QString::fromStdString(jv_name),
                    m_robot_state->getVariablePosition(vidx));
        }
    }

    config.mapSetValue("joint_tolerance", m_joint_tol_rad);
    config.mapSetValue("position_tolerance", m_pos_tol_m);
    config.mapSetValue("orientation_tolerance", m_rot_tol_rad);
    config.mapSetValue("workspace_frame", QString::fromStdString(m_workspace.header.frame_id));
    config.mapSetValue("workspace_min_x", m_workspace.min_corner.x);
    config.mapSetValue("workspace_min_y", m_workspace.min_corner.y);
    config.mapSetValue("workspace_min_z", m_workspace.min_corner.z);
    config.mapSetValue("workspace_max_x", m_workspace.max_corner.x);
    config.mapSetValue("workspace_max_y", m_workspace.max_corner.y);
    config.mapSetValue("workspace_max_z", m_workspace.max_corner.z);

    ROS_INFO("Saved model configuration");
}

void MoveGroupCommandModel::setPlannerName(const std::string& planner_name)
{
    if (planner_name == plannerName()) {
        return;
    }

    for (size_t i = 0; i < m_planner_interfaces.size(); ++i) {
        if (m_planner_interfaces[i].name == planner_name) {
            m_curr_planner_idx = (int)i;
            Q_EMIT configChanged();
            return;
        }
    }

    ROS_ERROR("Planner '%s' was not found in the planner descriptions", planner_name.c_str());
}

void MoveGroupCommandModel::setPlannerID(const std::string& planner_id)
{
    if (planner_id == plannerID()) {
        return;
    }

    if (m_curr_planner_idx == -1) {
        ROS_ERROR("No planner selected");
        return;
    }

    const moveit_msgs::PlannerInterfaceDescription& planner_desc =
            m_planner_interfaces[m_curr_planner_idx];
    for (size_t i = 0; i < planner_desc.planner_ids.size(); ++i) {
        const std::string& id = planner_desc.planner_ids[i];
        if (id == planner_id) {
            m_curr_planner_id_idx = (int)i;
            Q_EMIT configChanged();
            return;
        }
    }

    ROS_ERROR("Planner ID '%s' was not found in planner '%s'", planner_id.c_str(), planner_desc.name.c_str());
}

void MoveGroupCommandModel::setNumPlanningAttempts(int num_planning_attempts)
{
    if (m_num_planning_attempts != num_planning_attempts) {
        m_num_planning_attempts = num_planning_attempts;
        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::setAllowedPlanningTime(double allowed_planning_time_s)
{
    if (m_allowed_planning_time_s != allowed_planning_time_s) {
        m_allowed_planning_time_s = allowed_planning_time_s;
        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::setPlanningJointGroup(
    const std::string& joint_group_name)
{
    if (m_curr_joint_group_name != joint_group_name) {
        m_curr_joint_group_name = joint_group_name;
        reinitInteractiveMarkers();
        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::setJointVariable(int jidx, double value)
{
    if (!isRobotLoaded()) {
        ROS_WARN("Robot model not loaded");
        return;
    }

    if (jidx < 0 || jidx >= robotModel()->getVariableCount()) {
        ROS_WARN("Index passed to setJointVariable out of bounds: jidx = %d, variable count = %zu", jidx, robotModel()->getVariableCount());
        return;
    }

    if (m_robot_state->getVariablePosition(jidx) != value) {
        m_robot_state->setVariablePosition(jidx, value);

        // Why would this happen?
        if (m_robot_state->getVariablePosition(jidx) != value) {
            ROS_WARN("Attempt to set joint variable %d to %0.3f failed", jidx, value);
            return;
        }

        m_robot_state->updateLinkTransforms();
        updateInteractiveMarkers();

        updateRobotStateValidity();

        Q_EMIT robotStateChanged();
    }
}

void MoveGroupCommandModel::setJointVariable(
    const std::string& jv_name,
    double value)
{
    if (!isRobotLoaded()) {
        ROS_WARN("Robot model not loaded");
        return;
    }

    if (!hasVariable(*robotModel(), jv_name)) {
        ROS_WARN("Joint variable name passed to setJointVariable does not exist in Robot Model");
        return;
    }

    if (m_robot_state->getVariablePosition(jv_name) != value) {
        m_robot_state->setVariablePosition(jv_name, value);

        // Why would this happen?
        if (m_robot_state->getVariablePosition(jv_name) != value) {
            ROS_WARN("Attempt to set joint variable '%s' to %0.3f failed", jv_name.c_str(), value);
            return;
        }

        m_robot_state->updateLinkTransforms();
        updateInteractiveMarkers();

        updateRobotStateValidity();

        Q_EMIT robotStateChanged();
    }
}

void MoveGroupCommandModel::setGoalJointTolerance(double tol_deg)
{
    if (tol_deg != sbpl::utils::ToDegrees(m_joint_tol_rad)) {
        m_joint_tol_rad = sbpl::utils::ToRadians(tol_deg);
        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::setGoalPositionTolerance(double tol_m)
{
    if (tol_m != m_pos_tol_m) {
        m_pos_tol_m = tol_m;
        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::setGoalOrientationTolerance(double tol_deg)
{
    if (tol_deg != sbpl::utils::ToDegrees(m_rot_tol_rad)) {
        m_rot_tol_rad = sbpl::utils::ToRadians(tol_deg);
        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::setWorkspace(
    const moveit_msgs::WorkspaceParameters& ws)
{
    if (ws.header.frame_id != m_workspace.header.frame_id ||
        ws.min_corner.x != m_workspace.min_corner.x ||
        ws.min_corner.y != m_workspace.min_corner.y ||
        ws.min_corner.z != m_workspace.min_corner.z ||
        ws.max_corner.x != m_workspace.max_corner.x ||
        ws.max_corner.y != m_workspace.max_corner.y ||
        ws.max_corner.z != m_workspace.max_corner.z)
    {
        m_workspace.header.frame_id = ws.header.frame_id;

        // partial rejection of invalid workspace dimensions
        if (ws.min_corner.x < ws.max_corner.x) {
            m_workspace.min_corner.x = ws.min_corner.x;
            m_workspace.max_corner.x = ws.max_corner.x;
        }
        if (ws.min_corner.y < ws.max_corner.y) {
            m_workspace.min_corner.y = ws.min_corner.y;
            m_workspace.max_corner.y = ws.max_corner.y;
        }
        if (ws.min_corner.z < ws.max_corner.z) {
            m_workspace.min_corner.z = ws.min_corner.z;
            m_workspace.max_corner.z = ws.max_corner.z;
        }

        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::reinitCheckStateValidityService()
{
    m_check_state_validity_client.reset(new ros::ServiceClient);
    *m_check_state_validity_client =
            m_nh.serviceClient<moveit_msgs::GetStateValidity>(
                    "check_state_validity");
}

void MoveGroupCommandModel::reinitQueryPlannerInterfaceService()
{
    m_query_planner_interface_client.reset(new ros::ServiceClient);
    *m_query_planner_interface_client =
            m_nh.serviceClient<moveit_msgs::QueryPlannerInterfaces>(
                    "query_planner_interface");
}

void MoveGroupCommandModel::logRobotModelInfo(
    const moveit::core::RobotModel& rm) const
{
    ROS_INFO("Robot Model Name: %s", rm.getName().c_str());
    ROS_INFO("Robot Model Frame: %s", rm.getModelFrame().c_str());
    ROS_INFO("Root Link Name: %s", rm.getRootLinkName().c_str());
    ROS_INFO("Root Joint Name: %s", rm.getRootJointName().c_str());

    ROS_INFO("--- Robot Links ---");
    std::stack<std::pair<int, const moveit::core::LinkModel*>> links;
    links.push(std::make_pair(0, rm.getRootLink()));
    while (!links.empty()) {
        int depth;
        const moveit::core::LinkModel* lm;
        std::tie(depth, lm) = links.top();
        links.pop();

        std::string pad(depth, ' ');
        ROS_INFO("%s%s", pad.c_str(), lm->getName().c_str());

        for (const moveit::core::JointModel* jm : lm->getChildJointModels()) {
            links.push(std::make_pair(depth+1, jm->getChildLinkModel()));
        }
    }

    ROS_INFO("--- Robot Joints ---");
    std::stack<std::pair<int, const moveit::core::JointModel*>> joints;
    joints.push(std::make_pair(0, rm.getRootJoint()));
    while (!joints.empty()) {
        int depth;
        const moveit::core::JointModel* jm;
        std::tie(depth, jm) = joints.top();
        joints.pop();

        std::string pad(depth, ' ');
        ROS_INFO("%s%s", pad.c_str(), jm->getName().c_str());

        const moveit::core::LinkModel* lm = jm->getChildLinkModel();
        for (const moveit::core::JointModel* j : lm->getChildJointModels()) {
            joints.push(std::make_pair(depth + 1, j));
        }
    }

    ROS_INFO("--- Robot Joint Groups ---");

    const std::vector<const moveit::core::JointModelGroup*>& jmgs =
            rm.getJointModelGroups();
    for (const moveit::core::JointModelGroup* jmg : jmgs) {
        ROS_INFO("Name: %s", jmg->getName().c_str());
        ROS_INFO("  Chain: %s", jmg->isChain() ? "true" : "false");
        ROS_INFO("  Only Single-DoF Joints: %s", jmg->isSingleDOFJoints() ? "true" : "false");
        ROS_INFO("  End Effector: %s", jmg->isEndEffector() ? "true" : "false");
        if (jmg->isEndEffector()) {
            ROS_INFO("    End Effector Name: %s", jmg->getEndEffectorName().c_str());
        }
        ROS_INFO("  Maximum Extent: %0.3f", jmg->getMaximumExtent());
        ROS_INFO("  Active Joints:");
        for (const moveit::core::JointModel* jm : jmg->getActiveJointModels()) {
            ROS_INFO("    %s", jm->getName().c_str());
        }
        ROS_INFO("  Non-Active Joints:");
        for (const moveit::core::JointModel* jm : jmg->getJointModels()) {
            if (std::find(
                    jmg->getActiveJointModels().begin(),
                    jmg->getActiveJointModels().end(),
                    jm) ==
                jmg->getActiveJointModels().end())
            {
                ROS_INFO("    %s", jm->getName().c_str());
            }
        }
        ROS_INFO("  Attached End Effectors:");
        for (const std::string& name : jmg->getAttachedEndEffectorNames()) {
            ROS_INFO("    %s", name.c_str());
        }
        ROS_INFO("  Common Root: %s", jmg->getCommonRoot() ? jmg->getCommonRoot()->getName().c_str() : "null");
        ROS_INFO("  Links for Setting IK:");
        for (const moveit::core::LinkModel* lm : jmg->getLinkModels()) {
            if (jmg->canSetStateFromIK(lm->getName())) {
                ROS_INFO("    %s", lm->getName().c_str());
            }
        }
        ROS_INFO("  End Effector Tips:");
        std::vector<const moveit::core::LinkModel*> ee_tips;
        if (jmg->getEndEffectorTips(ee_tips)) {
            for (const moveit::core::LinkModel* ee_tip : ee_tips) {
                ROS_INFO("    %s", ee_tip->getName().c_str());
            }
        }
        ROS_INFO("  Tip Links:");
        for (const std::string& tip : getTipLinks(*jmg)) {
            ROS_INFO("    %s", tip.c_str());
        }
    }

    ROS_INFO("--- Joint Variables ---");

    for (size_t vind = 0; vind < rm.getVariableCount(); ++vind) {
        const std::string& var_name = rm.getVariableNames()[vind];
        const auto& var_bounds = rm.getVariableBounds(var_name);
        ROS_INFO("%s: { min: %f, max: %f, vel: %f, acc: %f }", var_name.c_str(), var_bounds.min_position_, var_bounds.max_position_, var_bounds.max_velocity_, var_bounds.max_acceleration_);
    }
}

void MoveGroupCommandModel::logPlanningSceneMonitor(
    const planning_scene_monitor::PlanningSceneMonitor& monitor) const
{
    ROS_INFO("Planning Scene Monitor Name: %s", monitor.getName().c_str());
    ROS_INFO("Default Attached Object Padding: %0.3f", monitor.getDefaultAttachedObjectPadding());
    ROS_INFO("Default Object Padding: %0.3f", monitor.getDefaultObjectPadding());
    ROS_INFO("Default Robot Scale: %0.3f", monitor.getDefaultRobotScale());
    ROS_INFO("Last Update Time: %0.3f", monitor.getLastUpdateTime().toSec());
    std::vector<std::string> monitored_topics;
    monitor.getMonitoredTopics(monitored_topics);
    ROS_INFO("Monitored Topics:");
    for (const std::string& topic : monitored_topics) {
        ROS_INFO("  %s", topic.c_str());
    }
    ROS_INFO("Planning Scene Publishing Frequency: %0.3f", monitor.getPlanningScenePublishingFrequency());
    ROS_INFO("Robot Model: %s", monitor.getRobotModel()->getName().c_str());
}

void MoveGroupCommandModel::reinitInteractiveMarkers()
{
    ROS_INFO("Reinitializing Interactive Markers");

    // clear all interactive markers
    m_im_server.clear();
    m_int_marker_names.clear();

    if (!robotModel()) {
        ROS_WARN("No robot model to initialize interactive markers from");
        // TODO: apply runondestruction idiom here
        m_im_server.applyChanges();
        return;
    }

    if (m_curr_joint_group_name.empty()) {
        ROS_WARN("No active joint group to initialize interactive markers from");
        m_im_server.applyChanges();
        return;
    }

    const moveit::core::JointModelGroup* jg =
            robotModel()->getJointModelGroup(m_curr_joint_group_name);
    if (!jg) {
        ROS_ERROR("Failed to retrieve joint group '%s'", m_curr_joint_group_name.c_str());
        m_im_server.applyChanges();
        return;
    }

    std::vector<const moveit::core::LinkModel*> tips;
    if (!jg->getEndEffectorTips(tips)) {
        ROS_ERROR("Failed to retrieve end effector tips for joint group '%s'", m_curr_joint_group_name.c_str());
        m_im_server.applyChanges();
        return;
    }

    for (const moveit::core::LinkModel* tip_link : tips) {
        ROS_INFO("Adding interactive marker for controlling pose of link %s", tip_link->getName().c_str());

        visualization_msgs::InteractiveMarker tip_marker;
        tip_marker.header.frame_id = robotModel()->getModelFrame();

        const Eigen::Affine3d& T_model_tip =
                m_robot_state->getGlobalLinkTransform(tip_link);
        tf::poseEigenToMsg(T_model_tip, tip_marker.pose);

        tip_marker.name = markerNameFromTipName(tip_link->getName());
        tip_marker.description = "ik control of link " + tip_link->getName();
        tip_marker.scale = 0.20f;

        visualization_msgs::InteractiveMarkerControl dof_control;
        dof_control.orientation_mode =
                visualization_msgs::InteractiveMarkerControl::INHERIT;
        dof_control.always_visible = false;
//        dof_control.description = "pose_control";

        dof_control.orientation.w = 1.0;
        dof_control.orientation.x = 1.0;
        dof_control.orientation.y = 0.0;
        dof_control.orientation.z = 0.0;

        dof_control.name = "rotate_x";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_x";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.orientation.w = 1.0;
        dof_control.orientation.x = 0.0;
        dof_control.orientation.y = 1.0;
        dof_control.orientation.z = 0.0;

        dof_control.name = "rotate_z";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_z";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.orientation.w = 1.0;
        dof_control.orientation.x = 0.0;
        dof_control.orientation.y = 0.0;
        dof_control.orientation.z = 1.0;

        dof_control.name = "rotate_y";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_y";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        auto feedback_fn = boost::bind(
                &MoveGroupCommandModel::processInteractiveMarkerFeedback,
                this,
                _1);
        m_im_server.insert(tip_marker, feedback_fn);
        m_int_marker_names.push_back(tip_marker.name);
    }

    m_im_server.applyChanges();

    updateInteractiveMarkers();
}

void MoveGroupCommandModel::updateInteractiveMarkers()
{
    for (const std::string& marker_name : m_int_marker_names) {
        // stuff the current pose
        std::string tip_link_name = tipNameFromMarkerName(marker_name);
        const Eigen::Affine3d& T_model_tip =
                m_robot_state->getGlobalLinkTransform(tip_link_name);

        geometry_msgs::Pose tip_pose;
        tf::poseEigenToMsg(T_model_tip, tip_pose);

        // update the pose of the interactive marker
        std_msgs::Header header;
        header.frame_id = robotModel()->getModelFrame();
        header.stamp = ros::Time(0);
        if (!m_im_server.setPose(marker_name, tip_pose, header)) {
            ROS_ERROR("Failed to set pose of interactive marker '%s'", marker_name.c_str());
        }
    }

    m_im_server.applyChanges();
}

void MoveGroupCommandModel::updateRobotStateValidity()
{
    assert(isRobotLoaded());

    if (!m_check_state_validity_client->isValid()) {
        reinitCheckStateValidityService();
    }

    if (m_check_state_validity_client->exists()) {
        moveit_msgs::GetStateValidity::Request req;
        moveit_msgs::GetStateValidity::Response res;

        moveit::core::robotStateToRobotStateMsg(*m_robot_state, req.robot_state);
        req.group_name = m_curr_joint_group_name;
        // req.constraints;

        if (!m_check_state_validity_client->call(req, res)) {
            ROS_WARN("Failed to call service '%s'", m_check_state_validity_client->getService().c_str());
            m_validity = boost::indeterminate;
        }
        else {
            if (res.valid) {
                m_validity = true;
            }
            else {
                m_validity = false;
            }
        }
    }
    else {
        m_validity = boost::indeterminate;
    }
}

void MoveGroupCommandModel::clearMoveGroupRequest()
{
    // workspace parameters dependent on the robot model
    // start-state possibly dependent on the robot model
    // constraints dependent on the robot model
    // planner id dependent on plugins loaded into move_group
    // group_name dependent on the loaded robot loaded
    // planning attempts, planning time, and scaling factor not-dependent on anything
}

bool MoveGroupCommandModel::fillWorkspaceParameters(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req)
{
    req.workspace_parameters.header.frame_id =
            m_workspace.header.frame_id.empty() ?
                    robotModel()->getModelFrame() : m_workspace.header.frame_id;
    req.workspace_parameters.header.seq = 0;
    req.workspace_parameters.header.stamp = now;
    req.workspace_parameters.min_corner = m_workspace.min_corner;
    req.workspace_parameters.max_corner = m_workspace.max_corner;
    return true;
}

bool MoveGroupCommandModel::fillStartState(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    moveit::core::RobotState robot_state(robotModel());
    if (!getActualState(robot_state)) {
        ROS_ERROR("Failed to get start state");
        return false;
    }

    sensor_msgs::JointState joint_state;
    moveit::core::robotStateToJointStateMsg(robot_state, joint_state);

    // copy over joint state from incoming sensor data
    req.start_state.joint_state.header.frame_id = "ooga booga";
    req.start_state.joint_state.header.seq = 0;
    req.start_state.joint_state.header.stamp = now;
    req.start_state.joint_state.name = joint_state.name;
    req.start_state.joint_state.position = joint_state.position;
    req.start_state.joint_state.velocity = joint_state.velocity;
    req.start_state.joint_state.effort = joint_state.effort;

    // TODO: keep another robot state around as the "current robot state" and
    // use it as the single point of access, rather than distinguishing here
    // between live sources of state and sources of state from the phantom

    // copy over joint

    // for each multi-dof joint
    //   find the transform between the parent link and the child link and set
    //   the transform accordingly here
    sensor_msgs::MultiDOFJointState& multi_dof_joint_state =
            req.start_state.multi_dof_joint_state;

    // WARN: So, I'm not really sure why a frame id is necessary here. I was
    // under the impression the multi-DOF transform would be the transform from
    // the parent link to the child link (which, I believe, agrees with
    // RobotState::getJointTransform), but there's some weird stuff going on in
    // moveit/robot_state/conversions.h that leads to believe maybe somewhere is
    // assuming the transform is from the model frame to the child link. Either
    // way, we're going to set the header of the joint state to be the model
    // frame, to avoid that conversion, and then be happy with the fact that the
    // only multi-dof joint in the pr2 model is the planar joint between /odom
    // and base_footprint. Here there be monsters.

    multi_dof_joint_state.header.frame_id = robotModel()->getModelFrame();
    multi_dof_joint_state.header.seq = 0;
    multi_dof_joint_state.header.stamp = now;

    const std::vector<const moveit::core::JointModel*>& multi_dof_joints =
            robotModel()->getMultiDOFJointModels();
    for (size_t jind = 0; jind < multi_dof_joints.size(); ++jind) {
        const moveit::core::JointModel* jm = multi_dof_joints[jind];

        multi_dof_joint_state.joint_names.push_back(jm->getName());

        const Eigen::Affine3d& joint_transform =
                m_robot_state->getJointTransform(jm);

        geometry_msgs::Transform trans;
        tf::transformEigenToMsg(joint_transform, trans);

        multi_dof_joint_state.transforms.push_back(trans);
    }

    // TODO: attach nailgun
    req.start_state.attached_collision_objects.clear();

    req.start_state.is_diff = false;
    return true;
}

bool MoveGroupCommandModel::fillGoalConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    moveit_msgs::Constraints goal_constraints;
    goal_constraints.name = "goal_constraints";

    const moveit::core::JointModelGroup* jmg =
            robotModel()->getJointModelGroup(group_name);
    if (!jmg->isChain()) {
        ROS_INFO("Planning for joint groups that are not kinematic chains is not supported");
        return false;
    }

    auto solver = jmg->getSolverInstance();
    const std::string& tip_link = solver->getTipFrames().front();
    ROS_INFO("Planning for pose of tip link '%s' of kinematic chain", tip_link.c_str());

    const Eigen::Vector3d target_offset(0.535, 0.0, 0.13);
    const Eigen::Affine3d& T_model_tip =
            m_robot_state->getGlobalLinkTransform(tip_link);
    const Eigen::Affine3d& T_model_tgtoff =
            T_model_tip * Eigen::Translation3d(target_offset);
    geometry_msgs::Pose tip_link_pose;
    tf::poseEigenToMsg(T_model_tip, tip_link_pose);

    // Position constraint on the tip link

    moveit_msgs::PositionConstraint goal_pos_constraint;

    goal_pos_constraint.header.frame_id = robotModel()->getModelFrame();
    goal_pos_constraint.header.seq = 0;
    goal_pos_constraint.header.stamp = now;

    goal_pos_constraint.link_name = tip_link;

    goal_pos_constraint.target_point_offset.x = target_offset.x();
    goal_pos_constraint.target_point_offset.y = target_offset.y();
    goal_pos_constraint.target_point_offset.z = target_offset.z();

    // specify region within 5cm of the goal position
    shape_msgs::SolidPrimitive tolerance_volume;
    tolerance_volume.type = shape_msgs::SolidPrimitive::SPHERE;
    tolerance_volume.dimensions = { m_pos_tol_m };
    goal_pos_constraint.constraint_region.primitives.push_back(tolerance_volume);
    goal_pos_constraint.constraint_region.primitive_poses.push_back(tip_link_pose);

    goal_pos_constraint.weight = 1.0;

    // Orientation constraint on the tip link

    // specify goal orientation within 5 degrees of the goal orientation
    moveit_msgs::OrientationConstraint goal_rot_constraint;

    goal_rot_constraint.header.frame_id = robotModel()->getModelFrame();
    goal_rot_constraint.header.seq = 0;
    goal_rot_constraint.header.stamp = now;

    goal_rot_constraint.orientation = tip_link_pose.orientation;

    goal_rot_constraint.link_name = tip_link;

    goal_rot_constraint.absolute_x_axis_tolerance = m_rot_tol_rad;
    goal_rot_constraint.absolute_y_axis_tolerance = m_rot_tol_rad;
    goal_rot_constraint.absolute_z_axis_tolerance = m_rot_tol_rad;

    goal_rot_constraint.weight = 1.0;

//    goal_constraints.joint_constraints;
    goal_constraints.position_constraints.push_back(goal_pos_constraint);
    goal_constraints.orientation_constraints.push_back(goal_rot_constraint);
//    goal_constraints.visibility_constraints;

    req.goal_constraints.push_back(goal_constraints);

    return true;
}

bool MoveGroupCommandModel::fillPathConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    return true;
}

bool MoveGroupCommandModel::fillTrajectoryConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    return true;
}

void MoveGroupCommandModel::logMotionPlanResponse(
    const moveit_msgs::MotionPlanResponse& res) const
{
    // trajectory_start
    const auto& trajectory_start = res.trajectory_start;
    ROS_INFO("trajectory_start:");
    const auto& start_joint_state = trajectory_start.joint_state;
    ROS_INFO("  joint_state:");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            start_joint_state.header.seq,
            start_joint_state.header.stamp.toSec(),
            start_joint_state.header.frame_id.c_str());
    ROS_INFO("    name: %zu", start_joint_state.name.size());
    ROS_INFO("    position: %zu", start_joint_state.position.size());
    ROS_INFO("    velocity: %zu", start_joint_state.velocity.size());
    ROS_INFO("    effort: %zu", start_joint_state.effort.size());
    const auto& start_multi_dof_joint_state = trajectory_start.multi_dof_joint_state;
    ROS_INFO("  multi_dof_joint_state:");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            start_multi_dof_joint_state.header.seq,
            start_multi_dof_joint_state.header.stamp.toSec(),
            start_multi_dof_joint_state.header.frame_id.c_str());
    ROS_INFO("    joint_names: %zu", start_multi_dof_joint_state.joint_names.size());
    ROS_INFO("    transforms: %zu", start_multi_dof_joint_state.transforms.size());
    ROS_INFO("    twist: %zu", start_multi_dof_joint_state.twist.size());
    ROS_INFO("    wrench: %zu", start_multi_dof_joint_state.wrench.size());
    const auto& start_attached_collision_objects = trajectory_start.attached_collision_objects;
    ROS_INFO("  attached_collision_objects: %zu", start_attached_collision_objects.size());
    ROS_INFO("  is_diff: %s", trajectory_start.is_diff ? "true" : "false");

    // group_name
    ROS_INFO("group_name: %s", res.group_name.c_str());

    // trajectory
    const auto& trajectory = res.trajectory;
    ROS_INFO("trajectory:");
    const auto& joint_trajectory = trajectory.joint_trajectory;
    ROS_INFO("  joint_trajectory: ");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            joint_trajectory.header.seq,
            joint_trajectory.header.stamp.toSec(),
            joint_trajectory.header.frame_id.c_str());
    ROS_INFO("    joint_names: %zu", joint_trajectory.joint_names.size());
    ROS_INFO("    points: %zu", joint_trajectory.points.size());
    const auto& multi_dof_joint_trajectory = trajectory.multi_dof_joint_trajectory;
    ROS_INFO("  multi_dof_joint_trajectory: ");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            multi_dof_joint_trajectory.header.seq,
            multi_dof_joint_trajectory.header.stamp.toSec(),
            multi_dof_joint_trajectory.header.frame_id.c_str());
    ROS_INFO("    joint_names: %zu", multi_dof_joint_trajectory.joint_names.size());
    ROS_INFO("    points: %zu", multi_dof_joint_trajectory.points.size());

    // planning_time
    ROS_INFO("planning_time: %0.6f", res.planning_time);

    // error_code
    ROS_INFO("error_code: { val: %s }", to_string(res.error_code).c_str());
}

void MoveGroupCommandModel::logMotionPlanResponse(
    const moveit_msgs::MoveGroupResult& res) const
{
    // error_code
    ROS_INFO("error_code: { val: %s }", to_string(res.error_code).c_str());

    // trajectory_start
    const auto& trajectory_start = res.trajectory_start;
    ROS_INFO("trajectory_start:");
    const auto& start_joint_state = trajectory_start.joint_state;
    ROS_INFO("  joint_state:");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            start_joint_state.header.seq,
            start_joint_state.header.stamp.toSec(),
            start_joint_state.header.frame_id.c_str());
    ROS_INFO("    name: %zu", start_joint_state.name.size());
    ROS_INFO("    position: %zu", start_joint_state.position.size());
    ROS_INFO("    velocity: %zu", start_joint_state.velocity.size());
    ROS_INFO("    effort: %zu", start_joint_state.effort.size());
    const auto& start_multi_dof_joint_state = trajectory_start.multi_dof_joint_state;
    ROS_INFO("  multi_dof_joint_state:");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            start_multi_dof_joint_state.header.seq,
            start_multi_dof_joint_state.header.stamp.toSec(),
            start_multi_dof_joint_state.header.frame_id.c_str());
    ROS_INFO("    joint_names: %zu", start_multi_dof_joint_state.joint_names.size());
    ROS_INFO("    transforms: %zu", start_multi_dof_joint_state.transforms.size());
    ROS_INFO("    twist: %zu", start_multi_dof_joint_state.twist.size());
    ROS_INFO("    wrench: %zu", start_multi_dof_joint_state.wrench.size());
    const auto& start_attached_collision_objects = trajectory_start.attached_collision_objects;
    ROS_INFO("  attached_collision_objects: %zu", start_attached_collision_objects.size());
    ROS_INFO("  is_diff: %s", trajectory_start.is_diff ? "true" : "false");

    // trajectory
    const auto& trajectory = res.planned_trajectory;
    ROS_INFO("trajectory:");
    const auto& joint_trajectory = trajectory.joint_trajectory;
    ROS_INFO("  joint_trajectory: ");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            joint_trajectory.header.seq,
            joint_trajectory.header.stamp.toSec(),
            joint_trajectory.header.frame_id.c_str());
    ROS_INFO("    joint_names: %zu", joint_trajectory.joint_names.size());
    ROS_INFO("    points: %zu", joint_trajectory.points.size());
    const auto& multi_dof_joint_trajectory = trajectory.multi_dof_joint_trajectory;
    ROS_INFO("  multi_dof_joint_trajectory: ");
    ROS_INFO("    header: { seq: %d, stamp: %0.3f, frame_id: %s }",
            multi_dof_joint_trajectory.header.seq,
            multi_dof_joint_trajectory.header.stamp.toSec(),
            multi_dof_joint_trajectory.header.frame_id.c_str());
    ROS_INFO("    joint_names: %zu", multi_dof_joint_trajectory.joint_names.size());
    ROS_INFO("    points: %zu", multi_dof_joint_trajectory.points.size());

    // TODO: executed trajectory

    // planning_time
    ROS_INFO("planning_time: %0.6f", res.planning_time);
}

bool MoveGroupCommandModel::sendMoveGroupPoseGoal(
    const std::string& group_name,
    const moveit_msgs::PlanningOptions& ops)
{
    if (!readyToPlan()) {
        return false;
    }

    if (!m_move_group_client->isServerConnected()) {
        ROS_ERROR("Connection Failure: Unable to send Move Group Action");
        return false;
    }

    moveit_msgs::MoveGroupGoal move_group_goal;

    const ros::Time now = ros::Time::now();

    moveit_msgs::MotionPlanRequest& req = move_group_goal.request;
    if (!fillWorkspaceParameters(now, group_name, req) ||
//        !fillStartState(now, group_name, req) ||
        !fillGoalConstraints(now, group_name, req) ||
        !fillPathConstraints(now, group_name, req) ||
        !fillTrajectoryConstraints(now, group_name, req))
    {
        return false;
    }

    req.start_state.is_diff = true;
    req.planner_id = plannerID();
    req.group_name = group_name;
    req.num_planning_attempts = m_num_planning_attempts;
    req.allowed_planning_time = m_allowed_planning_time_s;
    req.max_velocity_scaling_factor = 1.0;

    move_group_goal.planning_options = ops;

    auto result_callback = boost::bind(
            &MoveGroupCommandModel::moveGroupResultCallback, this, _1, _2);
    m_move_group_client->sendGoal(move_group_goal, result_callback);

    return true;
}

void MoveGroupCommandModel::moveGroupResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const moveit_msgs::MoveGroupResult::ConstPtr& result)
{
    const auto& res = *result;
    logMotionPlanResponse(res);
}

bool MoveGroupCommandModel::getActualState(
    moveit::core::RobotState& robot_state) const
{
    if (!m_scene_monitor) {
        ROS_WARN("No scene loaded");
        return false;
    }

    m_scene_monitor->lockSceneRead();

    if (!m_scene_monitor->getStateMonitor()->haveCompleteState()) {
        ROS_WARN("Missing joint values for robot state");
    }

    planning_scene::PlanningScenePtr scene = m_scene_monitor->getPlanningScene();

    const robot_state::RobotState& curr_state = scene->getCurrentState();
    sensor_msgs::JointState curr_joint_state;
    moveit::core::robotStateToJointStateMsg(curr_state, curr_joint_state);
    robot_state.setVariableValues(curr_joint_state);

    m_scene_monitor->unlockSceneRead();

    return true;
}

std::vector<std::string> MoveGroupCommandModel::getTipLinks(
    const moveit::core::JointModelGroup& jmg) const
{
    std::vector<std::string> tips;
    for (const moveit::core::JointModel* jm : jmg.getJointRoots()) {
        std::string tip;
        getTipLinks(jmg, *jm->getParentLinkModel(), tip, tips);
    }
    return tips;
}

void MoveGroupCommandModel::getTipLinks(
    const moveit::core::JointModelGroup& jmg,
    const moveit::core::LinkModel& link,
    std::string& tip,
    std::vector<std::string>& tips) const
{
    if (jmg.canSetStateFromIK(link.getName())) {
        tip = link.getName();
    }

    std::vector<const moveit::core::LinkModel*> child_links;
    for (const moveit::core::JointModel* cjm : link.getChildJointModels()) {
        if (jmg.hasJointModel(cjm->getName())) {
            child_links.push_back(cjm->getChildLinkModel());
        }
    }

    if (child_links.empty()) {
        tips.push_back(tip);
    }

    for (const moveit::core::LinkModel* clm : child_links) {
        std::string ctip;
        getTipLinks(jmg, *clm, ctip, tips);
    }
}

void MoveGroupCommandModel::processInteractiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg)
{
    ROS_DEBUG("Interactive marker feedback");
    ROS_DEBUG("  Marker: %s", msg->marker_name.c_str());
    ROS_DEBUG("  Control: %s", msg->control_name.c_str());
    ROS_DEBUG("  Event Type: %u", (unsigned)msg->event_type);

    switch (msg->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE:
        break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
        const moveit::core::JointModelGroup* jg =
                m_robot_state->getJointModelGroup(m_curr_joint_group_name);
        if (!jg) {
            ROS_ERROR("Failed to retrieve joint group '%s'", m_curr_joint_group_name.c_str());
            break;
        }

        // run ik from this tip link
        if (!m_robot_state->setFromIK(jg, msg->pose)) {
            // TODO: anything special here?
        }
        m_robot_state->updateLinkTransforms();
        updateInteractiveMarkers();
        Q_EMIT robotStateChanged();
    }   break;
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    default:
        break;
    }
    std::string tip_link_name = tipNameFromMarkerName(msg->marker_name);

}

void MoveGroupCommandModel::processSceneUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
{
    switch (type) {
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_NONE:
    {
        ROS_INFO("Planning Scene Update (None)");
    }   break;
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE:
    {
        ROS_INFO("Planning Scene Update (State)");
        updateAvailableFrames();
    }   break;
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS:
    {
        ROS_INFO("Planning Scene Update (Transforms)");
        updateAvailableFrames();
    }   break;
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY:
    {
        ROS_INFO("Planning Scene Update (Geometry)");
        updateAvailableFrames();
    }   break;
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE:
    {
        ROS_INFO("Planning Scene Update (All)");
        updateAvailableFrames();
    }   break;
    default:
    {
    }   break;
    }
}

std::string MoveGroupCommandModel::markerNameFromTipName(
    const std::string& tip_name) const
{
    return tip_name + "_controls";
}

std::string MoveGroupCommandModel::tipNameFromMarkerName(
    const std::string& marker_name) const
{
    return marker_name.substr(0, marker_name.rfind("_control"));
}

bool MoveGroupCommandModel::plannerIndicesValid(
    int planner_idx, int planner_id_idx) const
{
    const auto& planner_ifaces = plannerInterfaces();
    if (planner_idx >= 0 && planner_idx < planner_ifaces.size()) {
        const auto& planner_iface = planner_ifaces[planner_idx];
        if (planner_id_idx >= 0 &&
            planner_id_idx < planner_iface.planner_ids.size())
        {
            return true;
        }
    }
    return false;
}

bool MoveGroupCommandModel::hasVariable(
    const moveit::core::RobotModel& rm,
    const std::string& jv_name) const
{
    const auto& jv_names = rm.getVariableNames();
    return std::find(jv_names.begin(), jv_names.end(), jv_name) != jv_names.end();
}

bool MoveGroupCommandModel::updateAvailableFrames()
{
    auto transformer = m_scene_monitor->getTFClient();

    if (!transformer) {
        if (m_available_frames.empty()) {
            return false;
        }
        else {
            m_available_frames.clear();
            return true;
        }
    }
    else {
        std::vector<std::string> frames;
        transformer->getFrameStrings(frames);

        std::sort(frames.begin(), frames.end());

        std::vector<std::string> diff;
        std::set_symmetric_difference(
                m_available_frames.begin(), m_available_frames.end(),
                frames.begin(), frames.end(),
                std::back_inserter(diff));

        if (!diff.empty()) {
            ROS_INFO("Transforms Changed %zu -> %zu", m_available_frames.size(), frames.size());
            m_available_frames = std::move(frames);
            Q_EMIT availableFramesUpdated();
            return true;
        }
        else {
            return false;
        }
    }
}

} // namespace sbpl_interface
