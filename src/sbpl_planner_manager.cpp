#include "sbpl_planner_manager.h"

#include <moveit/planning_scene/planning_scene.h>

#include "collision_detector_allocator_sbpl.h"
#include "sbpl_planning_context.h"
#include "collision_world_sbpl.h"

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

const std::string DefaultPlanningAlgorithm = "ARA*";

SBPLPlannerManager::SBPLPlannerManager() :
    Base(),
    m_robot_model(),
    m_ns()
{
    ROS_INFO("Constructed SBPL Planner Manager");
}

SBPLPlannerManager::~SBPLPlannerManager()
{
    ROS_INFO("Destructed SBPL Planner Manager");
}

bool SBPLPlannerManager::initialize(
    const robot_model::RobotModelConstPtr& model,
    const std::string& ns)
{
    ROS_INFO("Initialized SBPL Planner Manager");
    ROS_INFO_STREAM("  Robot Model: " << model->getName());
    ROS_INFO_STREAM("  Namespace: " << ns);

    m_robot_model = model;
    m_ns = ns;

    if (!loadPlannerConfigurationMapping(*model)) {
        ROS_ERROR("Failed to load planner configurations");
        return false;
    }

    return true;
}

std::string SBPLPlannerManager::getDescription() const
{
    return "Search-Based Planning Algorithms";
}

void SBPLPlannerManager::getPlanningAlgorithms(
    std::vector<std::string>& algs) const
{
    algs.push_back("ARA*");
}

planning_interface::PlanningContextPtr SBPLPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    ROS_INFO("SBPLPlannerManager::getPlanningContext");

    planning_interface::PlanningContextPtr context;

    if (!canServiceRequest(req)) {
        ROS_WARN("Unable to service request");
        return context;
    }

    if (!planning_scene) {
        ROS_WARN("Planning Context is null");
        return context;
    }

    planning_scene::PlanningScenePtr diff_scene = planning_scene->diff();

    ///////////////////////////
    // Setup SBPL Robot Model
    ///////////////////////////

    SBPLPlannerManager* mutable_me = const_cast<SBPLPlannerManager*>(this);
    MoveItRobotModel* sbpl_model = mutable_me->getModelForGroup(req.group_name);
    if (!sbpl_model) {
        ROS_WARN("No SBPL Robot Model available for group '%s'", req.group_name.c_str());
        return context;
    }

    bool res = true;
    res &= sbpl_model->setPlanningScene(diff_scene);
    res &= sbpl_model->setPlanningFrame(diff_scene->getPlanningFrame());
    if (!res) {
        ROS_WARN("Failed to set SBPL Robot Model's planning scene or planning frame");
        return context;
    }

    // get the planner configuration for this group
    const auto& pcs = getPlannerConfigurations().find(req.group_name)->second;
    auto pit = pcs.config.find("use_sbpl_collision_checking");
    bool use_sbpl_cc = false;
    if (pit == pcs.config.end()) {
        ROS_WARN("No configuration found for parameter 'use_sbpl_collision_checking'. Using default collision checking");
    }
    else {
        use_sbpl_cc = (pit->second == "true");
    }

    ///////////////////////////////////////////////
    // Setup collision checking on Planning Scene
    ///////////////////////////////////////////////

    // set the correct collision checker
    if (use_sbpl_cc) {
        ROS_INFO("Using SBPL collision checker");
        if (!mutable_me->selectCollisionCheckerSBPL(*diff_scene, req.group_name)) {
            ROS_WARN("No Collision Checker available for group '%s'", req.group_name.c_str());
            return context;
        }
    }
    else {
        ROS_INFO("Using default collision checker");
    }

    logPlanningScene(*diff_scene);
    logMotionRequest(req);

    SBPLPlanningContext* sbpl_context = new SBPLPlanningContext(
            sbpl_model, "sbpl_planning_context", req.group_name);

    // find a configuration for this group + planner_id
    const planning_interface::PlannerConfigurationMap& pcm = getPlannerConfigurations();

    // merge group parameters and planning configuration parameters of the
    // appropriate planner type
    std::map<std::string, std::string> all_params;
    for (auto it = pcm.begin(); it != pcm.end(); ++it) {
        const std::string& name = it->first;
        const planning_interface::PlannerConfigurationSettings& pcs = it->second;
        const std::string& group_name = req.group_name;
        if (name.find(group_name) != std::string::npos) {
            auto iit = pcs.config.find("type");
            if (iit == pcs.config.end() || iit->second == req.planner_id) {
                all_params.insert(pcs.config.begin(), pcs.config.end());
            }
        }
    }
    sbpl_context->setPlannerConfiguration(all_params);

    sbpl_context->setPlanningScene(diff_scene);
    sbpl_context->setMotionPlanRequest(req);

    context.reset(sbpl_context);
    return context;
}

bool SBPLPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    ROS_INFO("SBPLPlannerManager::canServiceRequest()");

    auto pcit = getPlannerConfigurations().find(req.group_name);
    if (pcit == getPlannerConfigurations().end()) {
        ROS_WARN("No planner configuration found for group '%s'", req.group_name.c_str());
        return false;
    }

    // TODO: check for existence of workspace parameters frame
    // TODO: check for positive workspace volume
    if (req.workspace_parameters.header.frame_id.empty()) {
        ROS_WARN("SBPL planner requires explicit workspace");
        return false;
    }

    for (const moveit_msgs::Constraints& constraints : req.goal_constraints) {
        if (!constraints.joint_constraints.empty()) {
            return false;
        }

        if (!constraints.visibility_constraints.empty()) {
            return false;
        }
    }

    return true;
}

void SBPLPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pcs)
{
    Base::setPlannerConfigurations(pcs);

    ROS_INFO("SBPLPlannerManager::setPlannerConfigurations");
    ROS_INFO("Planner Configurations");
    for (const auto& entry : pcs) {
        ROS_INFO("  %s: { name: %s, group: %s }", entry.first.c_str(), entry.second.group.c_str(), entry.second.name.c_str());
        for (const auto& e : entry.second.config) {
            ROS_INFO("    %s: %s", e.first.c_str(), e.second.c_str());
        }
    }
}

void SBPLPlannerManager::terminate() const
{
    ROS_INFO("SBPLPlannerManager::terminate()");
}

void SBPLPlannerManager::logPlanningScene(
    const planning_scene::PlanningScene& scene) const
{
    ROS_INFO("Planning Scene");
    ROS_INFO("  Name: %s", scene.getName().c_str());
    ROS_INFO("  Has Parent: %s", scene.getParent() ? "true" : "false");
    ROS_INFO("  Has Robot Model: %s", scene.getRobotModel() ? "true" : "false");
    ROS_INFO("  Planning Frame: %s", scene.getPlanningFrame().c_str());
    ROS_INFO("  Active Collision Detector Name: %s", scene.getActiveCollisionDetectorName().c_str());
    ROS_INFO("  Has World: %s", scene.getWorld() ? "true" : "false");
    if (scene.getWorld()) {
        ROS_INFO("    size:  %zu", scene.getWorld()->size());
        ROS_INFO("    Object IDs: %zu", scene.getWorld()->getObjectIds().size());
        for (size_t oind; oind < scene.getWorld()->getObjectIds().size(); ++oind) {
            const std::string& object_id = scene.getWorld()->getObjectIds()[oind];
            ROS_INFO("      %s", object_id.c_str());
        }
    }
    ROS_INFO("  Has Collision World: %s", scene.getCollisionWorld() ? "true" : "false");
    ROS_INFO("  Has Collision Robot: %s", scene.getCollisionRobot() ? "true" : "false");
    ROS_INFO("  Current State:");

    const moveit::core::RobotState& current_state = scene.getCurrentState();
    for (size_t vind = 0; vind < current_state.getVariableCount(); ++vind) {
        ROS_INFO("    %s: %0.3f", current_state.getVariableNames()[vind].c_str(), current_state.getVariablePosition(vind));
    }

    ROS_INFO("Allowed collision matrix");
    scene.getAllowedCollisionMatrix().print(std::cout);
}

void SBPLPlannerManager::logMotionRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    ROS_INFO("Motion Plan Request");

    ROS_INFO("  workspace_parameters");
    ROS_INFO("    header");
    ROS_INFO_STREAM("      seq: " << req.workspace_parameters.header.seq);
    ROS_INFO_STREAM("      stamp: " << req.workspace_parameters.header.stamp);
    ROS_INFO_STREAM("      frame_id: " << req.workspace_parameters.header.frame_id.c_str());
    ROS_INFO("    min_corner");
    ROS_INFO_STREAM("      x: " << req.workspace_parameters.min_corner.x);
    ROS_INFO_STREAM("      y: " << req.workspace_parameters.min_corner.y);
    ROS_INFO_STREAM("      z: " << req.workspace_parameters.min_corner.z);
    ROS_INFO("    max_corner");
    ROS_INFO_STREAM("      x: " << req.workspace_parameters.max_corner.x);
    ROS_INFO_STREAM("      y: " << req.workspace_parameters.max_corner.y);
    ROS_INFO_STREAM("      z: " << req.workspace_parameters.max_corner.z);

    ROS_INFO("  start_state");
    ROS_INFO("    joint_state:");
    const sensor_msgs::JointState& joint_state = req.start_state.joint_state;
    for (size_t jind = 0; jind < joint_state.name.size(); ++jind) {
        ROS_INFO("      { name: %s, position: %0.3f }", joint_state.name[jind].c_str(), joint_state.position[jind]);
    }
    ROS_INFO("    multi_dof_joint_state");
    ROS_INFO("    attached_collision_objects: %zu", req.start_state.attached_collision_objects.size());
    ROS_INFO("    is_diff: %s", req.start_state.is_diff ? "true" : "false");

    ROS_INFO("  goal_constraints: %zu", req.goal_constraints.size());
    for (size_t cind = 0; cind < req.goal_constraints.size(); ++cind) {
        const moveit_msgs::Constraints& constraints = req.goal_constraints[cind];

        // joint constraints
        ROS_INFO("    joint_constraints: %zu", constraints.joint_constraints.size());
        for (size_t jcind = 0; jcind < constraints.joint_constraints.size(); ++jcind) {
            const moveit_msgs::JointConstraint& joint_constraint =
                    constraints.joint_constraints[jcind];
            ROS_INFO("      joint_name: %s, position: %0.3f, tolerance_above: %0.3f, tolerance_below: %0.3f, weight: %0.3f",
                    joint_constraint.joint_name.c_str(),
                    joint_constraint.position,
                    joint_constraint.tolerance_above,
                    joint_constraint.tolerance_below,
                    joint_constraint.weight);
        }

        // position constraints
        ROS_INFO("    position_constraints: %zu", constraints.position_constraints.size());
        for (size_t pcind = 0; pcind < constraints.position_constraints.size(); ++pcind) {
            const moveit_msgs::PositionConstraint pos_constraint = 
                    constraints.position_constraints[pcind];
            ROS_INFO("      header: { frame_id: %s, seq: %u, stamp: %0.3f }", pos_constraint.header.frame_id.c_str(), pos_constraint.header.seq, pos_constraint.header.stamp.toSec());
            ROS_INFO("      link_name: %s", pos_constraint.link_name.c_str());
            ROS_INFO("      target_point_offset: (%0.3f, %0.3f, %0.3f)", pos_constraint.target_point_offset.x, pos_constraint.target_point_offset.y, pos_constraint.target_point_offset.z);
            ROS_INFO("      constraint_region:");
            ROS_INFO("        primitives: %zu", pos_constraint.constraint_region.primitives.size());
            for (size_t pind = 0; pind < pos_constraint.constraint_region.primitives.size(); ++pind) {
                const shape_msgs::SolidPrimitive& prim = pos_constraint.constraint_region.primitives[pind];
                const geometry_msgs::Pose& pose = pos_constraint.constraint_region.primitive_poses[pind];
                ROS_INFO("          { type: %d, pose: { position: (%0.3f, %0.3f, %0.3f), orientation: (%0.3f, %0.3f, %0.3f, %0.3f) } }", prim.type, pose.position.x, pose.position.y, pose.position.y, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            }
            ROS_INFO("        meshes: %zu", pos_constraint.constraint_region.meshes.size());
        }

        // orientation constarints
        ROS_INFO("    orientation_constraints: %zu", constraints.orientation_constraints.size());
        for (size_t ocind = 0; ocind < constraints.orientation_constraints.size(); ++ocind) {
            const moveit_msgs::OrientationConstraint rot_constraint =
                    constraints.orientation_constraints[ocind];
                ROS_INFO("      header: { frame_id: %s, seq: %u, stamp: %0.3f }", rot_constraint.header.frame_id.c_str(), rot_constraint.header.seq, rot_constraint.header.stamp.toSec());
                ROS_INFO("      orientation: (%0.3f, %0.3f, %0.3f, %0.3f)", rot_constraint.orientation.w, rot_constraint.orientation.x, rot_constraint.orientation.y, rot_constraint.orientation.z);
                ROS_INFO("      link_name: %s", rot_constraint.link_name.c_str());
                ROS_INFO("      absolute_x_axis_tolerance: %0.3f", rot_constraint.absolute_x_axis_tolerance);
                ROS_INFO("      absolute_y_axis_tolerance: %0.3f", rot_constraint.absolute_y_axis_tolerance);
                ROS_INFO("      absolute_z_axis_tolerance: %0.3f", rot_constraint.absolute_z_axis_tolerance);
                ROS_INFO("      weight: %0.3f", rot_constraint.weight);
        }

        // visibility constraints
        ROS_INFO("    visibility_constraints: %zu", constraints.visibility_constraints.size());
    }

    ROS_INFO("  path_constraints");
    ROS_INFO("    joint_constraints: %zu", req.path_constraints.joint_constraints.size());
    ROS_INFO("    position_constraints: %zu", req.path_constraints.position_constraints.size());
    ROS_INFO("    orientation_constraints: %zu", req.path_constraints.orientation_constraints.size());
    ROS_INFO("    visibility_constraints: %zu", req.path_constraints.visibility_constraints.size());

    ROS_INFO("  trajectory_constraints");
    for (size_t cind = 0; cind < req.trajectory_constraints.constraints.size(); ++cind) {
        const moveit_msgs::Constraints& constraints = req.trajectory_constraints.constraints[cind];
        ROS_INFO("    joint_constraints: %zu", constraints.joint_constraints.size());
        ROS_INFO("    position_constraints: %zu", constraints.position_constraints.size());
        ROS_INFO("    orientation_constraints: %zu", constraints.orientation_constraints.size());
        ROS_INFO("    visibility_constraints: %zu", constraints.visibility_constraints.size());
    }
    
    ROS_INFO_STREAM("  planner_id: " << req.planner_id);
    ROS_INFO_STREAM("  group_name: " << req.group_name);
    ROS_INFO_STREAM("  num_planning_attempts: " << req.num_planning_attempts);
    ROS_INFO_STREAM("  allowed_planning_time: " << req.allowed_planning_time);
    ROS_INFO_STREAM("  max_velocity_scaling_factor: " << req.max_velocity_scaling_factor);
}

bool SBPLPlannerManager::loadPlannerConfigurationMapping(
    const moveit::core::RobotModel& model)
{
    ros::NodeHandle nh(m_ns);

    // map<string, pcs>
    planning_interface::PlannerConfigurationMap pcm;

    // gather settings for each planner
    PlannerSettingsMap planner_settings_map;
    if (!loadPlannerSettings(planner_settings_map)) {
        ROS_ERROR("Failed to load planner settings");
        return false;
    }

    ROS_INFO("Successfully loaded planner settings");

    // TODO: implement defaults for parameters
    const bool DefaultUseCollisionCheckingSBPL = true;
    const bool DefaultUseSnapMprimXYZRPY = false;
    const bool DefaultUseSnapMprimXYZ = false;
    const double DefaultIkMprimDistThresh = 0.2;
    const bool DefaultUseSnapRPY = false;
    const bool DefaultUseMprimsMultiRes = true;
    const double DefaultShortDistMprimsThresh = 0.4;
    const bool DefaultDiscretization = 2.0 * M_PI / 360.0;

    const char* known_group_param_names[] = {
        "use_sbpl_collision_checking",
        "discretization",
        "mprim_filename",
        "use_xyzrpy_snap_mprim",
        "use_xyz_snap_mprim",
        "ik_mprim_dist_thresh",
        "use_rpy_snap_mprim",
        "use_multi_res_mprims",
        "short_dist_mprims_thresh"
    };

    const std::vector<std::string>& joint_group_names =
            model.getJointModelGroupNames();
    for (size_t jind = 0; jind < joint_group_names.size(); ++jind) {
        const std::string& joint_group_name = joint_group_names[jind];
        if (!nh.hasParam(joint_group_name)) {
            ROS_WARN("No planning configuration for joint group '%s'", joint_group_name.c_str());
            continue;
        }

        ROS_INFO("Reading configuration for joint group '%s'", joint_group_name.c_str());

        XmlRpc::XmlRpcValue joint_group_cfg;
        if (!nh.getParam(joint_group_name, joint_group_cfg)) {
            ROS_ERROR("Failed to retrieve '%s'", joint_group_name.c_str());
            return false;
        }

        if (joint_group_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("'<joint_group> should be a map of parameter names to parameter values");
            return false;
        }

        ROS_INFO("Creating (group, planner) configurations");

        if (joint_group_cfg.hasMember("planner_config")) {
            XmlRpc::XmlRpcValue& group_planner_configs_cfg =
                    joint_group_cfg["planner_config"];
            if (group_planner_configs_cfg.getType() !=
                XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("'planner_configs' should be an array of strings");
                return false;
            }

            for (int pcind = 0;
                pcind < group_planner_configs_cfg.size();
                ++pcind)
            {
                XmlRpc::XmlRpcValue& group_planner_config =
                        group_planner_configs_cfg[pcind];
                if (group_planner_config.getType() !=
                    XmlRpc::XmlRpcValue::TypeString)
                {
                    ROS_ERROR("'planner_configs' should be an array of strings");
                    return false;
                }

                std::string planner_config_name =
                        (std::string)group_planner_configs_cfg[pcind];

                auto it = planner_settings_map.find(planner_config_name);
                if (it == planner_settings_map.end()) {
                    ROS_WARN("No planner settings exist for configuration '%s'", planner_config_name.c_str());
                }
                else {
                    // create a separate group of planner configuration settings for
                    // the joint group with this specific planner
                    planning_interface::PlannerConfigurationSettings pcs;
                    pcs.name =
                            joint_group_name + "[" + planner_config_name + "]";
                    pcs.group = joint_group_name;
                    pcs.config = it->second;
                    pcm[pcs.name] = pcs;
                }
            }
        }

        ROS_INFO("Creating group configuration");

        bool found_all = true;
        PlannerSettings known_settings;
        for (size_t pind = 0;
            pind < sizeof(known_group_param_names) / sizeof(const char*);
            ++pind)
        {
            const char* param_name = known_group_param_names[pind];
            if (!joint_group_cfg.hasMember(param_name)) {
                ROS_WARN("Group '%s' lacks parameter '%s'", joint_group_name.c_str(), param_name);
                found_all = false;
                break;
            }

            ROS_INFO("Converting parameter '%s' to string representation", param_name);
            XmlRpc::XmlRpcValue& param = joint_group_cfg[param_name];
            if (!xmlToString(param, known_settings[param_name])) {
                ROS_ERROR("Unsupported parameter type");
            }
            else {
                ROS_INFO("Converted parameter to '%s'", known_settings[param_name].c_str());
            }
        }

        if (found_all) {
            planning_interface::PlannerConfigurationSettings pcs;
            pcs.name = joint_group_name;
            pcs.group = joint_group_name;
            pcs.config = known_settings;
            pcm[pcs.name] = pcs;
        }
    }

    setPlannerConfigurations(pcm);
    return true;
}

bool SBPLPlannerManager::loadPlannerSettings(PlannerSettingsMap& out)
{
    ros::NodeHandle nh(m_ns);
    if (!nh.hasParam("planner_configs")) {
        return true;
    }

    PlannerSettingsMap planner_configs;

    XmlRpc::XmlRpcValue planner_configs_cfg;
    if (!nh.getParam("planner_configs", planner_configs_cfg)) {
        ROS_ERROR("Failed to retrieve 'planner_configs'");
        return false;
    }

    // planner_configs should be a mapping of planner configuration names to
    // another struct which is a mapping of parameter names (strings) to
    // parameter values (type known per-parameter)
    if (planner_configs_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("'planner_configs' section should be a map of planner configuration names to planner configurations (found type '%s')", xmlTypeToString(planner_configs_cfg.getType()));
        return false;
    }

    for (auto it = planner_configs_cfg.begin();
        it != planner_configs_cfg.end();
        ++it)
    {
        const std::string& planner_config_name = it->first;
        XmlRpc::XmlRpcValue& planner_settings_cfg = it->second;

        ROS_INFO("Reading configuration for '%s'", planner_config_name.c_str());

        if (planner_settings_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("Planner configuration should be a map of parameter names to values");
            return false;
        }

        PlannerSettings planner_settings;
        for (auto iit = planner_settings_cfg.begin();
            iit != planner_settings_cfg.end();
            ++iit)
        {
            const std::string& planner_setting_name = iit->first;
            XmlRpc::XmlRpcValue& planner_setting = iit->second;
            ROS_INFO("Reading value for parameter '%s'", planner_setting_name.c_str());
            if (!xmlToString(
                    planner_setting, planner_settings[planner_setting_name]))
            {
                ROS_ERROR("Unsupported parameter type");
            }
            // planner_settings filled if no error above
        }

        planner_configs[planner_config_name] = planner_settings;
    }

    out = std::move(planner_configs);
    return true;
}

bool SBPLPlannerManager::xmlToString(
    XmlRpc::XmlRpcValue& value, std::string& out) const
{
    switch (value.getType()) {
    case XmlRpc::XmlRpcValue::TypeString:
    {
        std::string string_param = (std::string)value;
        out = string_param;
    }   break;
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
        bool bool_param = (bool)value;
        out = bool_param ? "true" : "false";
    }   break;
    case XmlRpc::XmlRpcValue::TypeInt:
    {
        int int_param = (int)value;
        out = std::to_string(int_param);
    }   break;
    case XmlRpc::XmlRpcValue::TypeDouble:
    {
        double double_param = (double)value;
        out = std::to_string(double_param);
    }   break;
    case XmlRpc::XmlRpcValue::TypeArray:
    {
        ROS_WARN("You should implement reading in arrays");
        return false;
    }   break;
    default:
        return false;
    }

    return true;
}

MoveItRobotModel* SBPLPlannerManager::getModelForGroup(
    const std::string& group_name)
{
    auto it = m_sbpl_models.find(group_name);
    if (it == m_sbpl_models.end()) {
        auto ent = m_sbpl_models.insert(
                std::make_pair(group_name, MoveItRobotModel()));
        assert(ent.second);
        MoveItRobotModel* sbpl_model = &(ent.first->second);
        if (!sbpl_model->init(m_robot_model, group_name)) {
            m_sbpl_models.erase(ent.first);
            ROS_WARN("Failed to initialize SBPL Robot Model");
            return nullptr;
        }

        if (!sbpl_model->planningTipLink()) {
            m_sbpl_models.erase(ent.first);
            ROS_WARN("SBPL Plugin does not currently support joint groups without a tip link");
            return nullptr;
        }
        return sbpl_model;
    }
    else {
        return &it->second;
    }
}

bool SBPLPlannerManager::selectCollisionCheckerSBPL(
    planning_scene::PlanningScene& scene,
    const std::string& group_name)
{
    // check for a pre-existing collision detector for this group
    auto it = m_cc_allocators.find(group_name);
    if (it != m_cc_allocators.end()) {
        scene.setActiveCollisionDetector(it->second, true);
        return true;
    }

    // create a new collision detector
    auto cc = collision_detection::CollisionDetectorAllocatorSBPL::create();
    scene.setActiveCollisionDetector(cc, true);

    // initialize the collision world (assumed to now be the sbpl collision
    // world)
    collision_detection::CollisionWorldConstPtr cworld =
            scene.getCollisionWorld();

    const collision_detection::CollisionWorldSBPL* sbpl_cworld = 
            dynamic_cast<const collision_detection::CollisionWorldSBPL*>(
                    cworld.get());

    collision_detection::CollisionWorldSBPL* mutable_cworld = 
            const_cast<collision_detection::CollisionWorldSBPL*>(sbpl_cworld);

    if (!mutable_cworld) {
        ROS_ERROR("Collision World is not a Collision World SBPL");
        return false;
    }

    std::string robot_description;
    if (!ros::NodeHandle().getParam("robot_description", robot_description)) {
        ROS_ERROR("Failed to retrieve parameter 'robot_description'");
        return false;
    }

    sbpl::collision::CollisionModelConfig config;
    if (!sbpl::collision::CollisionModelConfig::Load(ros::NodeHandle(m_ns), config)) {
        ROS_ERROR("Failed to load collision model config");
        return false;
    }

    mutable_cworld->init(robot_description, group_name, config);

    auto ent = m_cc_allocators.insert(std::make_pair(group_name, cc));
    assert(ent.second);
    return true;
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbpl_interface::SBPLPlannerManager, planning_interface::PlannerManager);
