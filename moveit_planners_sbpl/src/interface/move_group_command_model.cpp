#include "move_group_command_model.h"

// standard includes
#include <assert.h>
#include <chrono>
#include <stack>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <ros/console.h>
#include <smpl/angles.h>

#include <moveit_planners_sbpl/interface/utils.h>

namespace sbpl_interface {

static const char* LOG = "move_group_command_model";

const char* to_cstring(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
{
    switch (type) {
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_NONE:
        return "UPDATE_NONE";
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE:
        return "UPDATE_STATE";
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS:
        return "UPDATE_TRANSFORMS";
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY:
        return "UPDATE_GEOMETRY";
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE:
        return "UPDATE_SCENE";
    default:
        return "<UNKNOWN>";
    }
}

MoveGroupCommandModel::MoveGroupCommandModel() :
    QObject(),
    m_robot_command_model(),
    m_nh(),
    m_command_robot_state_pub(),
    m_scene_monitor(),
    m_validity(boost::indeterminate),
    m_check_state_validity_client(),
    m_query_planner_interface_client(),
    m_move_group_client(),
    m_planner_interfaces(),
    m_curr_planner_idx(-1),
    m_curr_planner_id_idx(-1),
    m_available_frames(),
    m_joint_tol_rad(smpl::angles::to_radians(DefaultGoalJointTolerance_deg)),
    m_pos_tol_m(DefaultGoalPositionTolerance_m),
    m_rot_tol_rad(smpl::angles::to_radians(DefaultGoalOrientationTolerance_deg)),
    m_workspace(),
    m_num_planning_attempts(DefaultNumPlanningAttempts),
    m_allowed_planning_time_s(DefaultAllowedPlanningTime_s),
    m_curr_joint_group_name()
{
    m_command_robot_state_pub = m_nh.advertise<moveit_msgs::RobotState>(
            "command_robot_state", 1);

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
        } else {
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

    connect(&m_robot_command_model, SIGNAL(robotStateChanged()),
            this, SLOT(updateRobotState()));
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
        ROS_DEBUG_NAMED(LOG, "Robot model already loaded");
        return true;
    }

    ROS_DEBUG_NAMED(LOG, "Load robot model");

    {   // all the initialization steps that can fail
        std::string robot_description_key;
        if (!m_nh.searchParam(robot_description, robot_description_key)) {
            ROS_WARN("Parameter '%s' was not found on the param server", robot_description.c_str());
            return false;
        }

        using planning_scene_monitor::PlanningSceneMonitor;

        auto transformer = boost::make_shared<tf::TransformListener>();

        planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
        scene_monitor.reset(new PlanningSceneMonitor(robot_description, transformer));

        if (!scene_monitor) {
            ROS_ERROR("Failed to instantiate Planning Scene Monitor");
            return false;
        }

        ROS_DEBUG_NAMED(LOG, "Created new Planning Scene Monitor");

        auto robot_model = scene_monitor->getRobotModel();
        if (!robot_model) {
            ROS_ERROR("Planning Scene Monitor failed to parse robot description");
            return false;
        }

        ROS_DEBUG_NAMED(LOG, "Initialize Robot Command Model");

        if (!m_robot_command_model.load(robot_model)) {
            ROS_ERROR("Failed to load Robot Model");
            return false;
        }

        m_scene_monitor = std::move(scene_monitor);
    }

    m_scene_monitor->requestPlanningSceneState();
    auto update_fn = [this](
        planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
    {
//        ROS_DEBUG_NAMED(LOG, "Process scene update %s", to_cstring(type));
        switch (type) {
        case planning_scene_monitor::PlanningSceneMonitor::UPDATE_NONE:
            break;
        case planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE:
        case planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS:
        case planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY:
        case planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE:
            updateAvailableFrames();
            break;
        default:
            break;
        }
    };
    m_scene_monitor->addUpdateCallback(update_fn);
    m_scene_monitor->startSceneMonitor();
    m_scene_monitor->startStateMonitor();

    logPlanningSceneMonitor(*m_scene_monitor);

    if (!robotModel()->hasJointModelGroup(m_curr_joint_group_name)) {
        if (!robotModel()->getJointModelGroupNames().empty()) {
            m_curr_joint_group_name =
                    robotModel()->getJointModelGroupNames().front();
        }
    }
    // otherwise retain the selected joint group

//    ROS_INFO_STREAM(RobotModelInfo(robotModel()));

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

auto MoveGroupCommandModel::robotModel() const
    -> const moveit::core::RobotModelConstPtr&
{
    return m_robot_command_model.getRobotModel(); //m_robot_model;
}

auto MoveGroupCommandModel::robotState() const
    -> const moveit::core::RobotState*
{
    return m_robot_command_model.getRobotState();
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

bool MoveGroupCommandModel::planToGoalPose()
{
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = false;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = true;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    return sendMoveGroupPoseGoal(m_curr_joint_group_name, ops);
}

bool MoveGroupCommandModel::planToGoalConfiguration()
{
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = false;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = true;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    return sendMoveGroupConfigurationGoal(m_curr_joint_group_name, ops);
}

bool MoveGroupCommandModel::moveToGoalPose()
{
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = false;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = false;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    return sendMoveGroupPoseGoal(m_curr_joint_group_name, ops);
}

bool MoveGroupCommandModel::moveToGoalConfiguration()
{
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = false;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = false;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    return sendMoveGroupConfigurationGoal(m_curr_joint_group_name, ops);
}

bool MoveGroupCommandModel::copyCurrentState()
{
    if (!m_scene_monitor) {
        ROS_WARN("No scene loaded");
        return false;
    }

    planning_scene_monitor::LockedPlanningSceneRO scene(m_scene_monitor);

    if (!m_scene_monitor->getStateMonitor()->haveCompleteState()) {
        ROS_WARN("Missing joint values for robot state");
    }

    auto& curr_state = scene->getCurrentState();
    m_robot_command_model.setVariablePositions(
            curr_state.getVariablePositions());

    return true;
}

auto MoveGroupCommandModel::plannerInterfaces() const
    -> const std::vector<moveit_msgs::PlannerInterfaceDescription>&
{
    return m_planner_interfaces;
}

auto MoveGroupCommandModel::availableFrames() const
    -> const std::vector<std::string>&
{
    return m_available_frames;
}

auto MoveGroupCommandModel::robotDescription() const
    -> const std::string
{
    if (m_scene_monitor) {
        return m_scene_monitor->getRobotDescription();
    } else {
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
    } else {
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
    } else {
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
    return smpl::angles::to_degrees(m_joint_tol_rad);
}

double MoveGroupCommandModel::goalPositionTolerance() const
{
    return m_pos_tol_m;
}

double MoveGroupCommandModel::goalOrientationTolerance() const
{
    return smpl::angles::to_degrees(m_rot_tol_rad);
}

const moveit_msgs::WorkspaceParameters& MoveGroupCommandModel::workspace() const
{
    return m_workspace;
}

void MoveGroupCommandModel::load(const rviz::Config& config)
{
    ROS_DEBUG_NAMED(LOG, "Load MoveGroupCommandModel from config");

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
    float joint_tol_rad = smpl::angles::to_radians(DefaultGoalJointTolerance_deg);
    float pos_tol_m = DefaultGoalPositionTolerance_m;
    float rot_tol_rad = smpl::angles::to_radians(DefaultGoalOrientationTolerance_deg);
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
    ROS_INFO("  Joint Tolerance (deg): %0.3f", smpl::angles::to_degrees(joint_tol_rad));
    ROS_INFO("  Position Tolerance (m): %0.3f", pos_tol_m);
    ROS_INFO("  Orientation Tolerance (deg): %0.3f", rot_tol_rad);

    // set up the model using public member functions to fire off appropriate
    // signals

    bool robot_loaded = false;
    if (!robot_description.toStdString().empty()) {
        ROS_INFO("Loading robot using saved robot_description parameter name");
        robot_loaded = loadRobot(robot_description.toStdString());
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
                m_robot_command_model.setVariablePosition(jv_name, jv_pos);
            }
        }
    }
    setGoalJointTolerance(smpl::angles::to_degrees(joint_tol_rad));
    setGoalPositionTolerance(pos_tol_m);
    setGoalOrientationTolerance(smpl::angles::to_degrees(rot_tol_rad));

    moveit_msgs::WorkspaceParameters ws;
    auto it = std::find(
            m_available_frames.begin(), m_available_frames.begin(),
            ws_frame.toStdString());
    if (it != m_available_frames.end()) {
        ws.header.frame_id = ws_frame.toStdString();
    } else {
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
    auto* robot_state = m_robot_command_model.getRobotState();
    if (robot_state) {
        for (int vidx = 0; vidx < robot_state->getVariableCount(); ++vidx) {
            auto& jv_name = robot_state->getVariableNames()[vidx];
            phantom_state_config.mapSetValue(
                    QString::fromStdString(jv_name),
                    robot_state->getVariablePosition(vidx));
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

    auto& planner_desc = m_planner_interfaces[m_curr_planner_idx];
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
        Q_EMIT configChanged();
    }
}

void MoveGroupCommandModel::setGoalJointTolerance(double tol_deg)
{
    if (tol_deg != smpl::angles::to_degrees(m_joint_tol_rad)) {
        m_joint_tol_rad = smpl::angles::to_radians(tol_deg);
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
    if (tol_deg != smpl::angles::to_degrees(m_rot_tol_rad)) {
        m_rot_tol_rad = smpl::angles::to_radians(tol_deg);
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

void MoveGroupCommandModel::updateRobotState()
{
    updateRobotStateValidity();
    notifyCommandStateChanged();
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

void MoveGroupCommandModel::updateRobotStateValidity()
{
    ROS_DEBUG_NAMED(LOG, "Update robot state validity");

    // RobotCommandModel must be initialized, but the rest need not be

    if (!m_check_state_validity_client->isValid()) {
        reinitCheckStateValidityService();
    }

    if (m_check_state_validity_client->exists()) {
        moveit_msgs::GetStateValidity::Request req;
        moveit_msgs::GetStateValidity::Response res;

        auto* robot_state = m_robot_command_model.getRobotState();
        assert(robot_state != NULL);

        moveit::core::robotStateToRobotStateMsg(*robot_state, req.robot_state);
        req.group_name = m_curr_joint_group_name;

        if (!m_check_state_validity_client->call(req, res)) {
            ROS_WARN("Failed to call service '%s'", m_check_state_validity_client->getService().c_str());
            m_validity = boost::indeterminate;
        } else {
            if (res.valid) {
                m_validity = true;
            } else {
                m_validity = false;
                for (const auto& contact : res.contacts) {
                    ROS_INFO("Links '%s' and '%s' are in collision", contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
                }
                m_contacts = res.contacts;
            }
        }
    } else {
        m_validity = boost::indeterminate;
    }
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

bool MoveGroupCommandModel::fillPoseGoalConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    moveit_msgs::Constraints goal_constraints;
    goal_constraints.name = "goal_constraints";

    auto* jmg = robotModel()->getJointModelGroup(group_name);
    auto solver = jmg->getSolverInstance();
    if (!solver || solver->getTipFrames().empty()) {
        ROS_INFO("Maybe use one of these subgroups instead");
        for (auto& subgroup : jmg->getSubgroupNames()) {
            ROS_INFO("  %s", subgroup.c_str());
        }

        auto subgroup_name = "right_arm";
        jmg = robotModel()->getJointModelGroup(subgroup_name);
        if (!jmg) {
            return false;
        }

        solver = jmg->getSolverInstance();
        if (!solver || solver->getTipFrames().empty()) {
            return false;
        }
    }

    if (!jmg->isChain()) {
        ROS_INFO("Planning for joint groups that are not kinematic chains is not supported");
        return false;
    }

    if (!solver) {
        ROS_ERROR("Unable to plan to pose for joint group '%s'. No tip link available", jmg->getName().c_str());
        return false;
    }

    if (solver->getTipFrames().empty()) {
        ROS_ERROR("Unable to plan to pose for joint group '%s'. No tip link available", jmg->getName().c_str());
        return false;
    }

    auto& tip_link = solver->getTipFrames().front();
    ROS_INFO("Planning for pose of tip link '%s' of kinematic chain", tip_link.c_str());

    auto* robot_state = m_robot_command_model.getRobotState();

    const Eigen::Vector3d target_offset(0.0, 0.0, 0.0);
    auto& T_model_tip = robot_state->getGlobalLinkTransform(tip_link);
    Eigen::Affine3d T_model_tgtoff = T_model_tip * Eigen::Translation3d(target_offset);
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

bool MoveGroupCommandModel::fillConfigurationGoalConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    moveit_msgs::Constraints constraints;
    constraints.name = "goal_constraints";

    auto* jmg = robotModel()->getJointModelGroup(group_name);
    auto* robot_state = m_robot_command_model.getRobotState();

    // make a joint constraint for each active joint in the joint model group
    for (auto* jm : jmg->getActiveJointModels()) {
        switch (jm->getType()) {
        case moveit::core::JointModel::JointType::FIXED:
            break;
        case moveit::core::JointModel::JointType::PRISMATIC: {
            moveit_msgs::JointConstraint joint_constraint;
            joint_constraint.joint_name = jm->getName();
            joint_constraint.position = robot_state->getVariablePosition(jm->getName());
            joint_constraint.tolerance_above = m_pos_tol_m;
            joint_constraint.tolerance_below = m_pos_tol_m;
            joint_constraint.weight = 1.0;

            constraints.joint_constraints.push_back(joint_constraint);
        }   break;
        case moveit::core::JointModel::JointType::REVOLUTE: { // fall-through
            moveit_msgs::JointConstraint joint_constraint;
            joint_constraint.joint_name = jm->getName();
            joint_constraint.position = robot_state->getVariablePosition(jm->getName());
            joint_constraint.tolerance_above = m_joint_tol_rad;
            joint_constraint.tolerance_below = m_joint_tol_rad;
            joint_constraint.weight = 1.0;

            constraints.joint_constraints.push_back(joint_constraint);
        }   break;
        case moveit::core::JointModel::JointType::PLANAR: {
            for (const std::string& var_name : jm->getVariableNames()) {
                moveit_msgs::JointConstraint joint_constraint;
                joint_constraint.joint_name = var_name;
                joint_constraint.position = robot_state->getVariablePosition(var_name);
                if (var_name == "theta") {
                    joint_constraint.tolerance_above = m_joint_tol_rad;
                    joint_constraint.tolerance_below = m_joint_tol_rad;
                } else {
                    joint_constraint.tolerance_above = m_pos_tol_m;
                    joint_constraint.tolerance_below = m_pos_tol_m;
                }
                joint_constraint.weight = 1.0;
                constraints.joint_constraints.push_back(joint_constraint);
            }
        }   break;
        case moveit::core::JointModel::JointType::FLOATING: {
            ROS_WARN("Skipping floating joint '%s' in joint group '%s'", jm->getName().c_str(), group_name.c_str());
        }   break;
        }
    }

    req.goal_constraints.push_back(constraints);

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
    ROS_INFO("error_code: { val: %s }", to_cstring(res.error_code));
}

void MoveGroupCommandModel::logMotionPlanResponse(
    const moveit_msgs::MoveGroupResult& res) const
{
    // error_code
    ROS_INFO("error_code: { val: %s }", to_cstring(res.error_code));

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
        !fillPoseGoalConstraints(now, group_name, req) ||
        !fillPathConstraints(now, group_name, req) ||
        !fillTrajectoryConstraints(now, group_name, req))
    {
        return false;
    }

    req.start_state.is_diff = false;
    req.planner_id = plannerID();
    req.group_name = group_name;
    req.num_planning_attempts = m_num_planning_attempts;
    req.allowed_planning_time = m_allowed_planning_time_s;
    req.max_velocity_scaling_factor = 1.0;

    move_group_goal.planning_options = ops;

    auto result_callback = [this](
        const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::MoveGroupResult::ConstPtr& result)
    {
        return moveGroupResultCallback(state, result);
    };

    m_move_group_client->sendGoal(move_group_goal, result_callback);

    return true;
}

bool MoveGroupCommandModel::sendMoveGroupConfigurationGoal(
    const std::string& group_name,
    const moveit_msgs::PlanningOptions& ops)
{
    if (!readyToPlan()) {
        return false;
    }

    moveit_msgs::MoveGroupGoal move_group_goal;

    const ros::Time now = ros::Time::now();

    moveit_msgs::MotionPlanRequest& req = move_group_goal.request;
    if (!fillWorkspaceParameters(now, group_name, req) ||
        !fillConfigurationGoalConstraints(now, group_name, req) ||
        !fillPathConstraints(now, group_name, req) ||
        !fillTrajectoryConstraints(now, group_name, req))
    {
        return false;
    }

    req.start_state.is_diff = false;
    req.planner_id = plannerID();
    req.group_name = group_name;
    req.num_planning_attempts = m_num_planning_attempts;
    req.allowed_planning_time = m_allowed_planning_time_s;
    req.max_velocity_scaling_factor = 1.0;

    move_group_goal.planning_options = ops;

    auto result_callback = [this](
        const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::MoveGroupResult::ConstPtr& result)
    {
        return moveGroupResultCallback(state, result);
    };

    m_move_group_client->sendGoal(move_group_goal, result_callback);

    return true;
}

void MoveGroupCommandModel::moveGroupResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const moveit_msgs::MoveGroupResult::ConstPtr& result)
{
    if (result) {
        const auto& res = *result;
        logMotionPlanResponse(res);
    }
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
        } else {
            m_available_frames.clear();
            return true;
        }
    } else {
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
        } else {
            return false;
        }
    }
}

void MoveGroupCommandModel::notifyCommandStateChanged()
{
    moveit_msgs::RobotState msg;
    moveit::core::robotStateToRobotStateMsg(
            *m_robot_command_model.getRobotState(),
            msg);
    m_command_robot_state_pub.publish(msg);
    Q_EMIT robotStateChanged();
}

} // namespace sbpl_interface
