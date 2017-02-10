#include "sbpl_planner_manager.h"

#include <leatherman/print.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include "sbpl_planning_context.h"
#include <moveit_planners_sbpl/collision_detector_allocator_sbpl.h>
#include <moveit_planners_sbpl/collision_world_sbpl.h>

static const char* xmlTypeToString(XmlRpc::XmlRpcValue::Type type)
{
    switch (type) {
    case XmlRpc::XmlRpcValue::TypeInvalid:
        return "Invalid";
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return "Boolean";
    case XmlRpc::XmlRpcValue::TypeInt:
        return "Int";
    case XmlRpc::XmlRpcValue::TypeDouble:
        return "Double";
    case XmlRpc::XmlRpcValue::TypeString:
        return "String";
    case XmlRpc::XmlRpcValue::TypeDateTime:
        return "DateTime";
    case XmlRpc::XmlRpcValue::TypeBase64:
        return "Base64";
    case XmlRpc::XmlRpcValue::TypeArray:
        return "Array";
    case XmlRpc::XmlRpcValue::TypeStruct:
        return "Struct";
    default:
        return "Unrecognized";
    }
}

namespace sbpl_interface {

// pp = planning plugin
static const char* PP_LOGGER = "planning";

SBPLPlannerManager::SBPLPlannerManager() :
    Base(),
    m_robot_model(),
    m_viz()
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Constructed SBPL Planner Manager");
    sbpl::viz::set_visualizer(&m_viz);
}

SBPLPlannerManager::~SBPLPlannerManager()
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Destructed SBPL Planner Manager");
    if (sbpl::viz::visualizer() == &m_viz) {
        sbpl::viz::unset_visualizer();
    }
}

bool SBPLPlannerManager::initialize(
    const robot_model::RobotModelConstPtr& model,
    const std::string& ns)
{
    ROS_INFO_NAMED(PP_LOGGER, "Initialize SBPL Planner Manager");
    ROS_INFO_NAMED(PP_LOGGER, "  Robot Model: %s", model->getName().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Namespace: %s", ns.c_str());

    m_robot_model = model;

    ros::NodeHandle nh(ns);
    if (!loadPlannerConfigurationMapping(nh, *model)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load planner configurations");
        return false;
    }

    ROS_INFO_NAMED(PP_LOGGER, "Initialized SBPL Planner Manager");
    return true;
}

std::string SBPLPlannerManager::getDescription() const
{
    return "Search-Based Planning Algorithms";
}

void SBPLPlannerManager::getPlanningAlgorithms(
    std::vector<std::string>& algs) const
{
    const auto& pcm = getPlannerConfigurations();
    for (const auto& entry : pcm) {
        algs.push_back(entry.first);
    }
}

planning_interface::PlanningContextPtr SBPLPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Get SBPL Planning Context");

    planning_interface::PlanningContextPtr context;

    if (!canServiceRequest(req)) {
        ROS_WARN_NAMED(PP_LOGGER, "Unable to service request");
        return context;
    }

    if (!planning_scene) {
        ROS_WARN_NAMED(PP_LOGGER, "Planning Scene is null");
        return context;
    }

    ///////////////////////////
    // Setup SBPL Robot Model
    ///////////////////////////

    SBPLPlannerManager* mutable_me = const_cast<SBPLPlannerManager*>(this);
    MoveItRobotModel* sbpl_model = mutable_me->getModelForGroup(req.group_name);
    if (!sbpl_model) {
        ROS_WARN_NAMED(PP_LOGGER, "No SBPL Robot Model available for group '%s'", req.group_name.c_str());
        return context;
    }

    std::string planning_link = selectPlanningLink(req);
    if (planning_link.empty()) {
        ROS_INFO_NAMED(PP_LOGGER, "Clear the planning link");
    } else {
        ROS_INFO_NAMED(PP_LOGGER, "Set planning link to '%s'", planning_link.c_str());
    }

    if (!sbpl_model->setPlanningLink(planning_link)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to set planning link to '%s'", planning_link.c_str());
        return context;
    }

    bool res = true;
    res &= sbpl_model->setPlanningScene(planning_scene);
    res &= sbpl_model->setPlanningFrame(planning_scene->getPlanningFrame());
    if (!res) {
        ROS_WARN_NAMED(PP_LOGGER, "Failed to set SBPL Robot Model's planning scene or planning frame");
        return context;
    }

    ///////////////////////////////////////////
    // Initialize a new SBPL Planning Context
    ///////////////////////////////////////////

//    logPlanningScene(*planning_scene);
    logMotionPlanRequest(req);

    SBPLPlanningContext* sbpl_context = new SBPLPlanningContext(
            sbpl_model, "sbpl_planning_context", req.group_name);

    // find a configuration for this group + planner_id
    const planning_interface::PlannerConfigurationMap& pcm = getPlannerConfigurations();

    // merge parameters from global group parameters and parameters for the
    // selected planning configuration
    std::map<std::string, std::string> all_params;
    for (auto it = pcm.begin(); it != pcm.end(); ++it) {
        const std::string& name = it->first;
        const planning_interface::PlannerConfigurationSettings& pcs = it->second;
        const std::string& group_name = req.group_name;
        if (name == group_name) {
            all_params.insert(pcs.config.begin(), pcs.config.end());
        } else if (name == req.planner_id) {
            all_params.insert(pcs.config.begin(), pcs.config.end());
        }
    }

    if (!sbpl_context->init(all_params)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to initialize SBPL Planning Context");
        delete sbpl_context;
        return context;
    }

    sbpl_context->setPlanningScene(planning_scene);
    sbpl_context->setMotionPlanRequest(req);

    context.reset(sbpl_context);
    return context;
}

bool SBPLPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    ROS_DEBUG_NAMED(PP_LOGGER, "SBPLPlannerManager::canServiceRequest()");

    if (req.allowed_planning_time < 0.0) {
        ROS_WARN_NAMED(PP_LOGGER, "allowed_planning_time must be non-negative");
        return false;
    }

    // check for a configuration for the requested group
    auto pcit = getPlannerConfigurations().find(req.group_name);
    if (pcit == getPlannerConfigurations().end()) {
        ROS_WARN_NAMED(PP_LOGGER, "No planner configuration found for group '%s'", req.group_name.c_str());
        return false;
    }

    std::vector<std::string> available_algs;
    getPlanningAlgorithms(available_algs);
    std::cout << "Available algorithms\n";
    for(auto it = available_algs.begin(); it != available_algs.end(); it++)
    {
        std::cout << *it << "\n";
    }
    if (std::find(
            available_algs.begin(), available_algs.end(), req.planner_id) ==
                    available_algs.end())
    {
        ROS_WARN_NAMED(PP_LOGGER, "SBPL planner does not support the '%s' algorithm", req.planner_id.c_str());
        return false;
    }

    // guard against unsupported constraints in the underlying interface
    std::string why;
    if (!sbpl::motion::PlannerInterface::SupportsGoalConstraints(
            req.goal_constraints, why))
    {
        ROS_ERROR_NAMED(PP_LOGGER, "goal constraints not supported (%s)", why.c_str());
        return false;
    }

    // TODO: ...an unfortunate moveit plugin truth
    if (!req.path_constraints.position_constraints.empty() ||
        !req.path_constraints.orientation_constraints.empty() ||
        !req.path_constraints.joint_constraints.empty() ||
        !req.path_constraints.visibility_constraints.empty())
    {
        ROS_WARN_NAMED(PP_LOGGER, "SBPL planner does not support path constraints");
        return false;
    }

    if (!req.trajectory_constraints.constraints.empty()) {
        ROS_WARN_NAMED(PP_LOGGER, "SBPL planner does not support trajectory constraints");
        return false;
    }

    // TODO: check start state for existence of state for our robot model

    // TODO: check for existence of workspace parameters frame? Would this make
    // this call tied to an explicit planning scene?
    if (req.workspace_parameters.header.frame_id.empty()) {
        ROS_WARN_NAMED(PP_LOGGER, "SBPL planner requires workspace parameters to have a valid frame");
        return false;
    }

    // check for positive workspace volume
    const auto& min_corner = req.workspace_parameters.min_corner;
    const auto& max_corner = req.workspace_parameters.max_corner;
    if (min_corner.x > max_corner.x ||
        min_corner.y > max_corner.y ||
        min_corner.z > max_corner.z)
    {
        std::stringstream reasons;
        if (min_corner.x > max_corner.x) {
            reasons << "negative length";
        }
        if (min_corner.y > max_corner.y) {
            reasons << (reasons.str().empty() ? "" : " ") << "negative width";
        }
        if (min_corner.z > max_corner.z) {
            reasons << (reasons.str().empty() ? "" : " ") << "negative height";
        }
        ROS_WARN_NAMED(PP_LOGGER, "SBPL planner requires valid workspace (%s)", reasons.str().c_str());
        return false;
    }

    return true;
}

void SBPLPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pcs)
{
    Base::setPlannerConfigurations(pcs);

    ROS_INFO_NAMED(PP_LOGGER, "Planner Configurations");
    for (const auto& entry : pcs) {
        ROS_INFO_NAMED(PP_LOGGER, "  %s: { name: %s, group: %s }", entry.first.c_str(), entry.second.name.c_str(), entry.second.group.c_str());
        for (const auto& e : entry.second.config) {
            ROS_INFO_NAMED(PP_LOGGER, "    %s: %s", e.first.c_str(), e.second.c_str());
        }
    }
}

void SBPLPlannerManager::logPlanningScene(
    const planning_scene::PlanningScene& scene) const
{
    ROS_INFO_NAMED(PP_LOGGER, "Planning Scene");
    ROS_INFO_NAMED(PP_LOGGER, "  Name: %s", scene.getName().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Has Parent: %s", scene.getParent() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Has Robot Model: %s", scene.getRobotModel() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Planning Frame: %s", scene.getPlanningFrame().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Active Collision Detector Name: %s", scene.getActiveCollisionDetectorName().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Has World: %s", scene.getWorld() ? "true" : "false");
    if (scene.getWorld()) {
        ROS_INFO_NAMED(PP_LOGGER, "    size:  %zu", scene.getWorld()->size());
        ROS_INFO_NAMED(PP_LOGGER, "    Object IDs: %zu", scene.getWorld()->getObjectIds().size());
        for (auto oit = scene.getWorld()->begin();
            oit != scene.getWorld()->end(); ++oit)
        {
            const std::string& object_id = oit->first;
            ROS_INFO_NAMED(PP_LOGGER, "      %s", object_id.c_str());
        }
    }
    ROS_INFO_NAMED(PP_LOGGER, "  Has Collision World: %s", scene.getCollisionWorld() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Has Collision Robot: %s", scene.getCollisionRobot() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Current State:");

    const moveit::core::RobotState& current_state = scene.getCurrentState();
    for (size_t vind = 0; vind < current_state.getVariableCount(); ++vind) {
        ROS_INFO_NAMED(PP_LOGGER, "    %s: %0.3f", current_state.getVariableNames()[vind].c_str(), current_state.getVariablePosition(vind));
    }

//    ROS_INFO_NAMED(PP_LOGGER, "Allowed collision matrix");
//    scene.getAllowedCollisionMatrix().print(std::cout);
}

void SBPLPlannerManager::logMotionPlanRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Motion Plan Request");

    ROS_DEBUG_NAMED(PP_LOGGER, "  workspace_parameters");
    ROS_DEBUG_NAMED(PP_LOGGER, "    header");
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      seq: " << req.workspace_parameters.header.seq);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      stamp: " << req.workspace_parameters.header.stamp);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      frame_id: \"" << req.workspace_parameters.header.frame_id.c_str() << "\"");
    ROS_DEBUG_NAMED(PP_LOGGER, "    min_corner");
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      x: " << req.workspace_parameters.min_corner.x);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      y: " << req.workspace_parameters.min_corner.y);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      z: " << req.workspace_parameters.min_corner.z);
    ROS_DEBUG_NAMED(PP_LOGGER, "    max_corner");
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      x: " << req.workspace_parameters.max_corner.x);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      y: " << req.workspace_parameters.max_corner.y);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      z: " << req.workspace_parameters.max_corner.z);

    ROS_DEBUG_NAMED(PP_LOGGER, "  start_state");
    ROS_DEBUG_NAMED(PP_LOGGER, "    joint_state:");
    const sensor_msgs::JointState& joint_state = req.start_state.joint_state;
    for (size_t jidx = 0; jidx < joint_state.name.size(); ++jidx) {
        ROS_DEBUG_NAMED(PP_LOGGER, "      { name: %s, position: %0.3f }", joint_state.name[jidx].c_str(), joint_state.position[jidx]);
    }
    ROS_DEBUG_NAMED(PP_LOGGER, "    multi_dof_joint_state");
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state = req.start_state.multi_dof_joint_state;
    ROS_DEBUG_NAMED(PP_LOGGER, "      header: { seq: %d, stamp: %0.3f, frame_id: \"%s\" }",
            multi_dof_joint_state.header.seq,
            multi_dof_joint_state.header.stamp.toSec(),
            multi_dof_joint_state.header.frame_id.c_str());
    for (size_t jidx = 0; jidx < multi_dof_joint_state.joint_names.size(); ++jidx) {
        const std::string& joint_name = multi_dof_joint_state.joint_names[jidx];
        const geometry_msgs::Transform& transform = multi_dof_joint_state.transforms[jidx];
        ROS_DEBUG_NAMED(PP_LOGGER, "      { joint_names: %s, transform: %s }", joint_name.c_str(), to_string(transform).c_str());
    }

    ROS_DEBUG_NAMED(PP_LOGGER, "    attached_collision_objects: %zu", req.start_state.attached_collision_objects.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    is_diff: %s", req.start_state.is_diff ? "true" : "false");

    ROS_DEBUG_NAMED(PP_LOGGER, "  goal_constraints: %zu", req.goal_constraints.size());
    for (size_t cind = 0; cind < req.goal_constraints.size(); ++cind) {
        const moveit_msgs::Constraints& constraints = req.goal_constraints[cind];

        // joint constraints
        ROS_DEBUG_NAMED(PP_LOGGER, "    joint_constraints: %zu", constraints.joint_constraints.size());
        for (size_t jcind = 0; jcind < constraints.joint_constraints.size(); ++jcind) {
            const moveit_msgs::JointConstraint& joint_constraint =
                    constraints.joint_constraints[jcind];
            ROS_DEBUG_NAMED(PP_LOGGER, "      joint_name: %s, position: %0.3f, tolerance_above: %0.3f, tolerance_below: %0.3f, weight: %0.3f",
                    joint_constraint.joint_name.c_str(),
                    joint_constraint.position,
                    joint_constraint.tolerance_above,
                    joint_constraint.tolerance_below,
                    joint_constraint.weight);
        }

        // position constraints
        ROS_DEBUG_NAMED(PP_LOGGER, "    position_constraints: %zu", constraints.position_constraints.size());
        for (size_t pcind = 0; pcind < constraints.position_constraints.size(); ++pcind) {
            const moveit_msgs::PositionConstraint pos_constraint =
                    constraints.position_constraints[pcind];
            ROS_DEBUG_NAMED(PP_LOGGER, "      header: { frame_id: %s, seq: %u, stamp: %0.3f }", pos_constraint.header.frame_id.c_str(), pos_constraint.header.seq, pos_constraint.header.stamp.toSec());
            ROS_DEBUG_NAMED(PP_LOGGER, "      link_name: %s", pos_constraint.link_name.c_str());
            ROS_DEBUG_NAMED(PP_LOGGER, "      target_point_offset: (%0.3f, %0.3f, %0.3f)", pos_constraint.target_point_offset.x, pos_constraint.target_point_offset.y, pos_constraint.target_point_offset.z);
            ROS_DEBUG_NAMED(PP_LOGGER, "      constraint_region:");
            ROS_DEBUG_NAMED(PP_LOGGER, "        primitives: %zu", pos_constraint.constraint_region.primitives.size());
            for (size_t pind = 0; pind < pos_constraint.constraint_region.primitives.size(); ++pind) {
                const shape_msgs::SolidPrimitive& prim = pos_constraint.constraint_region.primitives[pind];
                const geometry_msgs::Pose& pose = pos_constraint.constraint_region.primitive_poses[pind];
                ROS_DEBUG_NAMED(PP_LOGGER, "          { type: %d, pose: { position: (%0.3f, %0.3f, %0.3f), orientation: (%0.3f, %0.3f, %0.3f, %0.3f) } }", prim.type, pose.position.x, pose.position.y, pose.position.y, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            }
            ROS_DEBUG_NAMED(PP_LOGGER, "        meshes: %zu", pos_constraint.constraint_region.meshes.size());
        }

        // orientation constarints
        ROS_DEBUG_NAMED(PP_LOGGER, "    orientation_constraints: %zu", constraints.orientation_constraints.size());
        for (size_t ocind = 0; ocind < constraints.orientation_constraints.size(); ++ocind) {
            const moveit_msgs::OrientationConstraint rot_constraint =
                    constraints.orientation_constraints[ocind];
                ROS_DEBUG_NAMED(PP_LOGGER, "      header: { frame_id: %s, seq: %u, stamp: %0.3f }", rot_constraint.header.frame_id.c_str(), rot_constraint.header.seq, rot_constraint.header.stamp.toSec());
                ROS_DEBUG_NAMED(PP_LOGGER, "      orientation: (%0.3f, %0.3f, %0.3f, %0.3f)", rot_constraint.orientation.w, rot_constraint.orientation.x, rot_constraint.orientation.y, rot_constraint.orientation.z);
                ROS_DEBUG_NAMED(PP_LOGGER, "      link_name: %s", rot_constraint.link_name.c_str());
                ROS_DEBUG_NAMED(PP_LOGGER, "      absolute_x_axis_tolerance: %0.3f", rot_constraint.absolute_x_axis_tolerance);
                ROS_DEBUG_NAMED(PP_LOGGER, "      absolute_y_axis_tolerance: %0.3f", rot_constraint.absolute_y_axis_tolerance);
                ROS_DEBUG_NAMED(PP_LOGGER, "      absolute_z_axis_tolerance: %0.3f", rot_constraint.absolute_z_axis_tolerance);
                ROS_DEBUG_NAMED(PP_LOGGER, "      weight: %0.3f", rot_constraint.weight);
        }

        // visibility constraints
        ROS_DEBUG_NAMED(PP_LOGGER, "    visibility_constraints: %zu", constraints.visibility_constraints.size());
    }

    ROS_DEBUG_NAMED(PP_LOGGER, "  path_constraints");
    ROS_DEBUG_NAMED(PP_LOGGER, "    joint_constraints: %zu", req.path_constraints.joint_constraints.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    position_constraints: %zu", req.path_constraints.position_constraints.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    orientation_constraints: %zu", req.path_constraints.orientation_constraints.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    visibility_constraints: %zu", req.path_constraints.visibility_constraints.size());

    ROS_DEBUG_NAMED(PP_LOGGER, "  trajectory_constraints");
    for (size_t cind = 0; cind < req.trajectory_constraints.constraints.size(); ++cind) {
        const moveit_msgs::Constraints& constraints = req.trajectory_constraints.constraints[cind];
        ROS_DEBUG_NAMED(PP_LOGGER, "    joint_constraints: %zu", constraints.joint_constraints.size());
        ROS_DEBUG_NAMED(PP_LOGGER, "    position_constraints: %zu", constraints.position_constraints.size());
        ROS_DEBUG_NAMED(PP_LOGGER, "    orientation_constraints: %zu", constraints.orientation_constraints.size());
        ROS_DEBUG_NAMED(PP_LOGGER, "    visibility_constraints: %zu", constraints.visibility_constraints.size());
    }

    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "  planner_id: " << req.planner_id);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "  group_name: " << req.group_name);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "  num_planning_attempts: " << req.num_planning_attempts);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "  allowed_planning_time: " << req.allowed_planning_time);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "  max_velocity_scaling_factor: " << req.max_velocity_scaling_factor);
}

/// Load the mapping from planner configuration name to planner configuration
/// settings.
bool SBPLPlannerManager::loadPlannerConfigurationMapping(
    const ros::NodeHandle& nh,
    const moveit::core::RobotModel& model)
{
    PlannerSettingsMap search_settings;
    if (!loadSettingsMap(nh, "search_configs", search_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load search settings");
        return false;
    }
    PlannerSettingsMap heuristic_settings;
    if (!loadSettingsMap(nh, "heuristic_configs", heuristic_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load heuristic settings");
        return false;
    }
    PlannerSettingsMap graph_settings;
    if (!loadSettingsMap(nh, "graph_configs", graph_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load graph settings");
        return false;
    }
    PlannerSettingsMap shortcut_settings;
    if (!loadSettingsMap(nh, "shortcut_configs", shortcut_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load shortcut settings");
        return false;
    }

    ROS_DEBUG_NAMED(PP_LOGGER, "Successfully loaded planner settings");

    planning_interface::PlannerConfigurationMap pcm;

    const char* req_group_params[] = { };

    for (const std::string& group_name : model.getJointModelGroupNames()) {
        if (!nh.hasParam(group_name)) {
            ROS_WARN_NAMED(PP_LOGGER, "No planning configuration for joint group '%s'", group_name.c_str());
            continue;
        }

        ROS_DEBUG_NAMED(PP_LOGGER, "Read configuration for joint group '%s'", group_name.c_str());

        // read group_name -> group config
        XmlRpc::XmlRpcValue joint_group_cfg;
        if (!nh.getParam(group_name, joint_group_cfg)) {
            ROS_ERROR_NAMED(PP_LOGGER, "Failed to retrieve '%s'", group_name.c_str());
            return false;
        }

        if (joint_group_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_NAMED(PP_LOGGER, "'%s' should be a map of group names to group settings", group_name.c_str());
            return false;
        }

        ROS_DEBUG_NAMED(PP_LOGGER, "Create (group, planner) configurations");

        // read array of planner configurations
        if (!joint_group_cfg.hasMember("planner_configs")) {
            ROS_WARN("No planner configurations specified for group '%s'", group_name.c_str());
            continue;
        }

        XmlRpc::XmlRpcValue& planner_configs_cfg = joint_group_cfg["planner_configs"];
        if (planner_configs_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_NAMED(PP_LOGGER, "'planner_configs' should be a map of names to planner configurations (actual: %s)", xmlTypeToString(planner_configs_cfg.getType()));
            return false;
        }

        for (auto pcit = planner_configs_cfg.begin(); pcit != planner_configs_cfg.end(); ++pcit) {
            const std::string& pc_name = pcit->first;
            XmlRpc::XmlRpcValue& planner_config = pcit->second;
            if (planner_config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR_NAMED(PP_LOGGER, "planner config should be a map from config type to config");
                return false;
            }

            const char *required_keys[] =
            {
                "search_config",
                "heuristic_config",
                "graph_config",
                "shortcut_config"
            };

            if (std::any_of(
                    required_keys,
                    required_keys + sizeof(required_keys) / sizeof(const char*),
                    [&](const char *key)
                    {
                        return !planner_config.hasMember(key);
                    }))
            {
                ROS_ERROR("planner config lacks required keys");
                for (int i = 0; i < sizeof(required_keys) / sizeof(const char*); ++i) {
                    const char* key = required_keys[i];
                    ROS_ERROR("Has '%s': %s", key, planner_config.hasMember(key) ? "true" : "false");
                }
                return false;
            }

            planning_interface::PlannerConfigurationSettings pcs;
            pcs.name = group_name + "[" + pc_name + "]";
            pcs.group = group_name;

            for (auto mit = planner_config.begin(); mit != planner_config.end(); ++mit) {
                if (mit->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
                    ROS_WARN_NAMED(PP_LOGGER, "planner configuration value is not a string");
                    continue;
                }
                const std::string &cfgkey(mit->first);
                const std::string &cfgval(mit->second);
                // squash search, heuristic, graph, and shortcut configurations
                // into combined config
                if (cfgkey == "graph_config") {
                    auto it = graph_settings.find(cfgval);
                    if (it == graph_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No graph settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else if (cfgkey == "heuristic_config") {
                    auto it = heuristic_settings.find(cfgval);
                    if (it == heuristic_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No heuristic settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else if (cfgkey == "search_config") {
                    auto it = search_settings.find(cfgval);
                    if (it == search_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No search settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else if (cfgkey == "shortcut_config") {
                    auto it = shortcut_settings.find(cfgval);
                    if (it == shortcut_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No shortcut settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else {
                    // anything else goes straight into the configuration map
                    pcs.config.insert({ cfgkey, cfgval });
                }
            }

            pcm[pcs.name] = pcs;
        }

        ROS_DEBUG_NAMED(PP_LOGGER, "Create group configuration");

        // Gather required parameters for the group, regardless of planner configuration
        PlannerSettings req_settings;
        if (std::all_of(
                req_group_params,
                req_group_params + sizeof(req_group_params) / sizeof(const char*),
                [&](const char* param_name)
                {
                    if (!joint_group_cfg.hasMember(param_name)) {
                        ROS_WARN_NAMED(PP_LOGGER, "Group '%s' lacks parameter '%s'", group_name.c_str(), param_name);
                        return false;
                    }

                    ROS_DEBUG_NAMED(PP_LOGGER, "Convert parameter '%s' to string representation", param_name);
                    XmlRpc::XmlRpcValue& param = joint_group_cfg[param_name];
                    if (!xmlToString(param, req_settings[param_name])) {
                        ROS_ERROR_NAMED(PP_LOGGER, "Unsupported parameter type");
                        return false;
                    }

                    ROS_DEBUG_NAMED(PP_LOGGER, "Converted parameter to '%s'", req_settings[param_name].c_str());
                    return true;
                }))
        {
            planning_interface::PlannerConfigurationSettings pcs;
            pcs.name = group_name;
            pcs.group = group_name;
            pcs.config = req_settings;
            pcm[pcs.name] = pcs;
        }
    }

    setPlannerConfigurations(pcm);
    return true;
}

bool SBPLPlannerManager::loadSettingsMap(
    const ros::NodeHandle& nh,
    const std::string& param_name,
    PlannerSettingsMap& settings)
{
    if (!nh.hasParam(param_name)) {
        return true;
    }

    PlannerSettingsMap planner_configs;

    XmlRpc::XmlRpcValue search_configs_cfg;
    if (!nh.getParam(param_name, search_configs_cfg)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to retrieve '%s'", param_name.c_str());
        return false;
    }

    // planner_configs should be a mapping of planner configuration names to
    // another struct which is a mapping of parameter names (strings) to
    // parameter values (type known per-parameter)
    if (search_configs_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR_NAMED(PP_LOGGER, "'planner_configs' section should be a map of planner configuration names to planner configurations (found type '%s')", xmlTypeToString(search_configs_cfg.getType()));
        return false;
    }

    for (auto it = search_configs_cfg.begin();
        it != search_configs_cfg.end();
        ++it)
    {
        const std::string& planner_config_name = it->first;
        XmlRpc::XmlRpcValue& planner_settings_cfg = it->second;

        ROS_DEBUG_NAMED(PP_LOGGER, "Read configuration for '%s'", planner_config_name.c_str());

        if (planner_settings_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_NAMED(PP_LOGGER, "Planner configuration should be a map of parameter names to values");
            return false;
        }

        PlannerSettings planner_settings;
        for (auto iit = planner_settings_cfg.begin();
            iit != planner_settings_cfg.end();
            ++iit)
        {
            const std::string& planner_setting_name = iit->first;
            XmlRpc::XmlRpcValue& planner_setting = iit->second;
            ROS_DEBUG_NAMED(PP_LOGGER, "Read value for parameter '%s'", planner_setting_name.c_str());
            if (!xmlToString(planner_setting, planner_settings[planner_setting_name])) {
                ROS_ERROR_NAMED(PP_LOGGER, "Unsupported parameter type");
            }
            // planner settings filled if no error above
        }

        planner_configs[planner_config_name] = planner_settings;
    }

    settings = std::move(planner_configs);
    return true;
}

MoveItRobotModel* SBPLPlannerManager::getModelForGroup(
    const std::string& group_name)
{
    auto it = m_sbpl_models.find(group_name);
    if (it == m_sbpl_models.end()) {
        auto ent = m_sbpl_models.insert(
                std::make_pair(group_name, std::make_shared<MoveItRobotModel>()));
        assert(ent.second);
        MoveItRobotModel* sbpl_model = ent.first->second.get();
        if (!sbpl_model->init(m_robot_model, group_name)) {
            m_sbpl_models.erase(ent.first);
            ROS_WARN_NAMED(PP_LOGGER, "Failed to initialize SBPL Robot Model");
            return nullptr;
        }

        ROS_INFO_NAMED(PP_LOGGER, "Created SBPL Robot Model for group '%s'", group_name.c_str());
        return sbpl_model;
    }
    else {
        ROS_DEBUG_NAMED(PP_LOGGER, "Use existing SBPL Robot Model for group '%s'", group_name.c_str());
        return it->second.get();
    }
}

std::string SBPLPlannerManager::selectPlanningLink(
    const planning_interface::MotionPlanRequest& req) const
{
    if (req.goal_constraints.empty()) {
        return std::string(); // doesn't matter, we'll bail out soon
    }

    const auto& goal_constraint = req.goal_constraints.front();
    // should've received one pose constraint for a single link, o/w
    // canServiceRequest would have complained
    if (!goal_constraint.position_constraints.empty()) {
        const auto& position_constraint = goal_constraint.position_constraints.front();
        return position_constraint.link_name;
    }

    // it's still useful to have a planning link for obstacle-based
    // heuristic information...inspect the joint model group

    const moveit::core::JointModelGroup* jmg =
            m_robot_model->getJointModelGroup(req.group_name);

    std::vector<std::string> ee_tips;
    if (jmg->getEndEffectorTips(ee_tips) && !ee_tips.empty()) {
        return ee_tips.front();
    }

    // TODO: can still pick a useful planning as the most descendant
    // link in the joint group...possibly prefer links for which an ik solver
    // is available

    if (!jmg->getLinkModelNames().empty()) {
        return jmg->getLinkModelNames().back();
    }

    return std::string(); // well...nothing we can do about this
}
bool SBPLPlannerManager::xmlToString(
    XmlRpc::XmlRpcValue& value, std::string& out) const
{
    switch (value.getType()) {
    case XmlRpc::XmlRpcValue::TypeString: {
        std::string string_param = (std::string)value;
        out = string_param;
    }   break;
    case XmlRpc::XmlRpcValue::TypeBoolean: {
        bool bool_param = (bool)value;
        out = bool_param ? "true" : "false";
    }   break;
    case XmlRpc::XmlRpcValue::TypeInt: {
        int int_param = (int)value;
        out = std::to_string(int_param);
    }   break;
    case XmlRpc::XmlRpcValue::TypeDouble: {
        double double_param = (double)value;
        out = std::to_string(double_param);
    }   break;
    case XmlRpc::XmlRpcValue::TypeArray: {
        std::stringstream ss;
        for (int i = 0; i < value.size(); ++i) {
            XmlRpc::XmlRpcValue& arr_value = value[i];
            switch (arr_value.getType()) {
            case XmlRpc::XmlRpcValue::TypeBoolean:
                ss << (bool)arr_value;
                break;
            case XmlRpc::XmlRpcValue::TypeInt:
                ss << (int)arr_value;
                break;
            case XmlRpc::XmlRpcValue::TypeDouble:
                ss << (double)arr_value;
                break;
            case XmlRpc::XmlRpcValue::TypeString:
                ss << (std::string)arr_value;
                break;
            default:
                ROS_ERROR_NAMED(PP_LOGGER, "Unsupported array member type (%s)", xmlTypeToString(arr_value.getType()));
                return false;
            }

            if (i != value.size() - 1) {
                ss << ' ';
            }
        }
        out = ss.str();
        return true;
    }   break;
    case XmlRpc::XmlRpcValue::TypeStruct: {
        std::stringstream ss;
        int i = 0;
        for (auto it = value.begin(); it != value.end(); ++it) {
            ss << it->first << ' ';
            XmlRpc::XmlRpcValue& struct_value = it->second;
            switch (struct_value.getType()) {
            case XmlRpc::XmlRpcValue::TypeBoolean:
                ss << (bool)struct_value;
                break;
            case XmlRpc::XmlRpcValue::TypeInt:
                ss << (int)struct_value;
                break;
            case XmlRpc::XmlRpcValue::TypeDouble:
                ss << (double)struct_value;
                break;
            case XmlRpc::XmlRpcValue::TypeString:
                ss << (std::string)struct_value;
                break;
            default:
                ROS_ERROR_NAMED(PP_LOGGER, "Unsupported struct member type (%s)", xmlTypeToString(struct_value.getType()));
                return false;
            }

            if (i != value.size() - 1) {
                ss << ' ';
            }

            ++i;
        }

        out = ss.str();
        return true;
    }   break;
    default: {
        return false;
    }   break;
    }

    return true;
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
        sbpl_interface::SBPLPlannerManager,
        planning_interface::PlannerManager);
