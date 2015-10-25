#include "sbpl_planner_manager.h"

#include <moveit/planning_scene/planning_scene.h>

#include "sbpl_planning_context.h"

namespace sbpl_interface {

const std::string DefaultPlanningAlgorithm = "ARA*";

SBPLPlannerManager::SBPLPlannerManager() :
    Base()
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

    if (!planning_scene) {
        ROS_WARN("Planning Context is null");
        return context;
    }

    logPlanningScene(*planning_scene);
    logMotionRequest(req);

    SBPLPlanningContext* sbpl_context =
            new SBPLPlanningContext("sbpl_planning_context", req.group_name);

    sbpl_context->setPlanningScene(planning_scene);
    sbpl_context->setMotionPlanRequest(req);

    context.reset(sbpl_context);
    return context;
}

bool SBPLPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    ROS_INFO("SBPLPlannerManager::canServiceRequest()");
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
    ROS_INFO("    Name: %s", scene.getName().c_str());
    ROS_INFO("    Has Parent: %s", scene.getParent() ? "true" : "false");
    ROS_INFO("    Has Robot Model: %s", scene.getRobotModel() ? "true" : "false");
    ROS_INFO("    Planning Frame: %s", scene.getPlanningFrame().c_str());
    ROS_INFO("    Active Collision Detector Name: %s", scene.getActiveCollisionDetectorName().c_str());
    ROS_INFO("    Has World: %s", scene.getWorld() ? "true" : "false");
    ROS_INFO("    Has Collision Robot: %s", scene.getCollisionRobot() ? "true" : "false");
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
    
    ROS_INFO("  trajectory_constraints");
    ROS_INFO_STREAM("  planner_id: " << req.planner_id);
    ROS_INFO_STREAM("  group_name: " << req.group_name);
    ROS_INFO_STREAM("  num_planning_attempts: " << req.num_planning_attempts);
    ROS_INFO_STREAM("  allowed_planning_time: " << req.allowed_planning_time);
    ROS_INFO_STREAM("  max_velocity_scaling_factor: " << req.max_velocity_scaling_factor);
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbpl_interface::SBPLPlannerManager, planning_interface::PlannerManager);
