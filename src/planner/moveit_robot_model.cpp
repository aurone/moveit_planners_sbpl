////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <moveit_planners_sbpl/planner/moveit_robot_model.h>

// standard includes
#include <math.h>

// system includes
#include <eigen_conversions/eigen_kdl.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/console/nonstd.h>
#include <tf_conversions/tf_eigen.h>

namespace sbpl_interface {

static const char* LOG = "model";

/// Initialize the MoveItRobotModel for the given MoveIt! Robot Model and the
/// group being planned for.
bool MoveItRobotModel::init(
    const robot_model::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    const std::string& ik_group_name)
{
    ROS_DEBUG_NAMED(LOG, "Initialize MoveIt! Robot Model");

    if (!robot_model) {
        ROS_ERROR("Robot Model is null");
        return false;
    }

    if (!robot_model->hasJointModelGroup(group_name)) {
        ROS_ERROR("Group '%s' does not exist within Robot Model", group_name.c_str());
        return false;
    }

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    robot_state->setToDefaultValues();

    // default frame to use for planning
    auto planning_frame = robot_model->getModelFrame();

    /////////////////////////////
    // Initialize Joint Groups //
    /////////////////////////////

    std::string real_ik_group_name;
    if (ik_group_name.empty()) {
        real_ik_group_name = group_name;
        // already know valid existence
    } else {
        real_ik_group_name = ik_group_name;
        if (!robot_model->hasJointModelGroup(real_ik_group_name)) {
            ROS_ERROR("Group '%s' does not exist within the Robot Model", real_ik_group_name.c_str());
            return false;
        }
    }

    auto* joint_group = robot_model->getJointModelGroup(group_name);
    auto* ik_group = robot_model->getJointModelGroup(real_ik_group_name);

    /////////////////////////////////////////////////////////////////////////
    // Cache the number, names, and index within robot state of all active //
    // variables in this group                                             //
    /////////////////////////////////////////////////////////////////////////

    auto active_joints = joint_group->getActiveJointModels();
    ROS_DEBUG_NAMED(LOG, "  Joint group has %zu active joints", active_joints.size());

    auto active_var_count = 0;
    std::vector<std::string> active_var_names;
    std::vector<int> active_var_indices;
    for (auto* joint : active_joints) {
        active_var_count += joint->getVariableCount();
        for (auto& var_name : joint->getVariableNames()) {
            active_var_names.push_back(var_name);
            active_var_indices.push_back(robot_model->getVariableIndex(var_name));
        }
    }
    ROS_DEBUG_NAMED(LOG, "Active Variable Count: %d", active_var_count);
    ROS_DEBUG_STREAM_NAMED(LOG, "Active Variable Names: " << active_var_names);
    ROS_DEBUG_STREAM_NAMED(LOG, "Active Variable Indices: " << active_var_indices);

    ///////////////////////////////////////////////////////////////
    // Cache the limits and properties of all planning variables //
    ///////////////////////////////////////////////////////////////

    std::vector<double> var_min_limits;
    std::vector<double> var_max_limits;
    std::vector<bool> var_continuous;
    std::vector<bool> var_bounded;
    std::vector<double> var_vel_limits;
    std::vector<double> var_acc_limits;

    var_min_limits.reserve(active_var_count);
    var_max_limits.reserve(active_var_count);
    var_continuous.reserve(active_var_count);
    var_bounded.reserve(active_var_count);
    var_vel_limits.reserve(active_var_count);
    var_acc_limits.reserve(active_var_count);
    for (auto& var_name : active_var_names) {
        auto vidx = robot_model->getVariableIndex(var_name);
        auto& var_bounds = robot_model->getVariableBounds(var_name);
        auto* jm = robot_model->getJointOfVariable(vidx);
        switch (jm->getType()) {
        case moveit::core::JointModel::UNKNOWN:
        case moveit::core::JointModel::JointType::FIXED:
            break; // shouldn't be here
        case moveit::core::JointModel::JointType::REVOLUTE:
            if (var_bounds.position_bounded_) {
                var_min_limits.push_back(var_bounds.min_position_);
                var_max_limits.push_back(var_bounds.max_position_);
                var_continuous.push_back(false);
                var_bounded.push_back(true);
            } else {
                // slight hack here? !position_bounded_ => continuous?
                var_min_limits.push_back(-M_PI);
                var_max_limits.push_back(M_PI);
                var_continuous.push_back(true);
                var_bounded.push_back(false);
            }
            break;
        case moveit::core::JointModel::JointType::PRISMATIC: {
            if (var_bounds.position_bounded_) {
                var_min_limits.push_back(var_bounds.min_position_);
                var_max_limits.push_back(var_bounds.max_position_);
                var_continuous.push_back(false);
                var_bounded.push_back(true);
            } else {
                var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                var_max_limits.push_back(std::numeric_limits<double>::infinity());
                var_continuous.push_back(false);
                var_bounded.push_back(false);
            }
        }   break;
        case moveit::core::JointModel::JointType::PLANAR: {
            if (var_bounds.position_bounded_) { // x and y variables
                var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                var_max_limits.push_back(std::numeric_limits<double>::infinity());
                var_continuous.push_back(false);
                var_bounded.push_back(false);
            } else { // theta variable
                var_min_limits.push_back(-M_PI);
                var_max_limits.push_back(M_PI);
                var_continuous.push_back(true);
                var_bounded.push_back(false);
            }
        }   break;
        case moveit::core::JointModel::JointType::FLOATING: {
            auto idx = var_name.find('/');
            if (idx != std::string::npos) {
                std::string local_var_name = var_name.substr(idx + 1);
                if (local_var_name == "trans_x") {
                    var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                    var_max_limits.push_back(std::numeric_limits<double>::infinity());
                    var_continuous.push_back(false);
                    var_bounded.push_back(false);
                } else if (local_var_name == "trans_y") {
                    var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                    var_max_limits.push_back(std::numeric_limits<double>::infinity());
                    var_continuous.push_back(false);
                    var_bounded.push_back(false);
                } else if (local_var_name == "trans_z") {
                    var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                    var_max_limits.push_back(std::numeric_limits<double>::infinity());
                    var_continuous.push_back(false);
                    var_bounded.push_back(false);
                } else if (local_var_name == "rot_w") {
                    var_min_limits.push_back(var_bounds.min_position_);
                    var_max_limits.push_back(var_bounds.max_position_);
                    var_continuous.push_back(false);
                    var_bounded.push_back(true);
                } else if (local_var_name == "rot_x") {
                    var_min_limits.push_back(var_bounds.min_position_);
                    var_max_limits.push_back(var_bounds.max_position_);
                    var_continuous.push_back(false);
                    var_bounded.push_back(true);
                } else if (local_var_name == "rot_y") {
                    var_min_limits.push_back(var_bounds.min_position_);
                    var_max_limits.push_back(var_bounds.max_position_);
                    var_continuous.push_back(false);
                    var_bounded.push_back(true);
                } else if (local_var_name == "rot_z") {
                    var_min_limits.push_back(var_bounds.min_position_);
                    var_max_limits.push_back(var_bounds.max_position_);
                    var_continuous.push_back(false);
                    var_bounded.push_back(true);
                } else {
                    ROS_ERROR("Unrecognized variable name '%s'", local_var_name.c_str());
                }
            } else {
                ROS_ERROR("Multi-DOF joint variable name missing '/' separator");
            }
        }   break;
        }

        if (var_bounds.velocity_bounded_) {
            var_vel_limits.push_back(var_bounds.max_velocity_);
        } else {
            var_vel_limits.push_back(0.0);
        }

        if (var_bounds.acceleration_bounded_) {
            var_acc_limits.push_back(var_bounds.max_acceleration_);
        } else {
            var_acc_limits.push_back(0.0);
        }
    }

    ROS_DEBUG_STREAM_NAMED(LOG, "Min Limits: " << var_min_limits);
    ROS_DEBUG_STREAM_NAMED(LOG, "Max Limits: " << var_max_limits);
    ROS_DEBUG_STREAM_NAMED(LOG, "Continuous: " << var_continuous);

    ///////////////////////////////////////////////////////////////////////////
    // Identify a default tip link to use for forward and inverse kinematics //
    // TODO: better default planning link (first tip link)                   //
    ///////////////////////////////////////////////////////////////////////////

    ROS_DEBUG_NAMED(LOG, "Look for planning link");

    // if we've requested a separate kinematics group, its solver must accept
    // the tip link

    const moveit::core::LinkModel* tip_link = NULL;
    if (joint_group->isChain()) {
        std::vector<const moveit::core::LinkModel*> tips;
        joint_group->getEndEffectorTips(tips);
        for (auto* tip : tips) {
            if (ik_group->canSetStateFromIK(tip->getName())) {
                tip_link = tip;
                ROS_DEBUG_NAMED(LOG, "  Found planning link '%s'", tip_link->getName().c_str());
                break;
            }
        }

        if (!tip_link) {
            if (tips.empty()) {
                ROS_WARN_ONCE("No end effector tip link present");
            } else {
                ROS_WARN_ONCE("Cannot set state from ik with respect to any available end effector tip links");
            }
        }
    }

    ////////////////////////////////////////////////
    // Cache redundant variable count and indices //
    ////////////////////////////////////////////////

    auto redundant_var_count = 0;
    std::vector<int> redundant_var_indices;

    auto solver = ik_group->getSolverInstance();
    if (solver) {
        std::vector<unsigned> redundant_joint_indices;
        solver->getRedundantJoints(redundant_joint_indices);
        redundant_var_count = (int)redundant_joint_indices.size();
        for (auto rvidx : redundant_joint_indices) {
            auto& joint_name = solver->getJointNames()[rvidx];
            auto it = std::find(begin(active_var_names), end(active_var_names), joint_name);
            if (it != end(active_var_names)) {
                redundant_var_indices.push_back(std::distance(begin(active_var_names), it));
            }
            // else hmm...
        }
    }

    auto redundant_ik_group = false;
    if (redundant_var_count > 0) redundant_ik_group = true;

    if (joint_group != ik_group) {
        // Find variables in the joint group that are not in the subgroup and
        // add those as the redundant variables

        auto full_group_variables = joint_group->getVariableNames();
        std::sort(begin(full_group_variables), end(full_group_variables));
        auto sub_group_variables = ik_group->getVariableNames();
        std::sort(begin(sub_group_variables), end(sub_group_variables));

        std::vector<std::string> full_group_only_variables;
        std::set_difference(
                begin(full_group_variables), end(full_group_variables),
                begin(sub_group_variables), end(sub_group_variables),
                std::back_inserter(full_group_only_variables));

        ROS_DEBUG_NAMED(LOG, "%zu variables outside of the sub-group", full_group_only_variables.size());
        for (auto& variable : full_group_only_variables) {
            ROS_DEBUG_NAMED(LOG, "  %s", variable.c_str());
            ++redundant_var_count;

            auto it = std::find(begin(active_var_names), end(active_var_names), variable);
            if (it == end(active_var_names)) {
                ROS_WARN("For some reason %s...", variable.c_str());
                continue;
            }

            redundant_var_indices.push_back(std::distance(begin(active_var_names), it));
        }

        ROS_DEBUG_NAMED(LOG, "Found %d redundancy variables", redundant_var_count);
        ROS_DEBUG_STREAM_NAMED(LOG, "Redundant variable indices: " << redundant_var_indices);
    }

    // TODO: determine whether the orientation solver is applicable for an
    // arbitrary joint group and the relevant links and joints that are part of
    // the spherical wrist

#ifdef PR2_WRIST_IK
    if (robot_model->getName() == "pr2") {
        if (group_name == "right_arm") {
            m_forearm_roll_link = "r_forearm_roll_link";
            m_wrist_flex_link = "r_wrist_flex_link";
            m_wrist_roll_link = "r_wrist_roll_link";
            m_wrist_flex_joint = "r_wrist_flex_joint";
        } else if (group_name == "left_arm") {
            m_forearm_roll_link = "l_forearm_roll_link";
            m_wrist_flex_link = "l_wrist_flex_link";
            m_wrist_roll_link = "l_wrist_roll_link";
            m_wrist_flex_joint = "l_wrist_flex_joint";
        }
    }

    if (!m_wrist_flex_joint.empty()) {
        auto& var_bounds = robot_model->getVariableBounds(m_wrist_flex_joint);

        double wrist_pitch_min = var_bounds.min_position_;
        double wrist_pitch_max = var_bounds.max_position_;
        ROS_DEBUG_NAMED(LOG, "Instantiating orientation solver with limits [%0.3f, %0.3f]", wrist_pitch_min, wrist_pitch_max);
        m_rpy_solver.reset(new smpl::RPYSolver(wrist_pitch_min, wrist_pitch_max));
    }
#endif

    //////////////////////////////
    // Complete the Robot Model //
    //////////////////////////////

    m_robot_model = robot_model;
    m_robot_state = std::move(robot_state);
    m_group_name = group_name;
    m_ik_group_name = std::move(real_ik_group_name);

    m_joint_group = joint_group;
    m_ik_group = ik_group;

    m_active_var_count = active_var_count;
    m_active_var_names = std::move(active_var_names);
    m_active_var_indices = std::move(active_var_indices);

    // TODO: why not just use the active variable names, from above, here
    // instead of gathering all variables names again

    std::vector<std::string> planning_variables;
    planning_variables.reserve(active_var_count);
    for (auto* joint : active_joints) {
        planning_variables.insert(
                end(planning_variables),
                begin(joint->getVariableNames()),
                end(joint->getVariableNames()));
    }
    setPlanningJoints(planning_variables);
    ROS_DEBUG_STREAM_NAMED(LOG, "Planning Variables: " << getPlanningJoints());

    m_var_min_limits = std::move(var_min_limits);
    m_var_max_limits = std::move(var_max_limits);
    m_var_continuous = std::move(var_continuous);
    m_var_bounded = std::move(var_bounded);
    m_var_vel_limits = std::move(var_vel_limits);
    m_var_acc_limits = std::move(var_acc_limits);

    m_tip_link = tip_link;

    m_redundant_ik_group = redundant_ik_group;
    m_redundant_var_count = redundant_var_count;
    m_redundant_var_indices = std::move(redundant_var_indices);

    m_planning_frame = std::move(planning_frame);

    return true;
}

bool MoveItRobotModel::initialized() const
{
    return m_joint_group != NULL;
}

auto MoveItRobotModel::moveitRobotModel() const -> moveit::core::RobotModelConstPtr
{
    return m_robot_model;
}

auto MoveItRobotModel::planningGroupName() const -> const std::string&
{
    return m_group_name;
}

auto MoveItRobotModel::planningJointGroup() const
    -> const moveit::core::JointModelGroup*
{
    return m_joint_group;
}

auto MoveItRobotModel::ikGroupName() const -> const std::string&
{
    return m_ik_group_name;
}

auto MoveItRobotModel::ikGroup() const -> const moveit::core::JointModelGroup*
{
    return m_ik_group;
}

/// Return the number of variables being planned for.
int MoveItRobotModel::activeVariableCount() const
{
    return m_active_var_count;
}

auto MoveItRobotModel::planningVariableNames() const
    -> const std::vector<std::string>&
{
    return m_active_var_names;
}

/// Return the indices into the moveit::core::RobotState joint variable vector
/// of the planning variables.
auto MoveItRobotModel::activeVariableIndices() const -> const std::vector<int>&
{
    return m_active_var_indices;
}

auto MoveItRobotModel::variableMinLimits() const -> const std::vector<double>&
{
    return m_var_min_limits;
}

auto MoveItRobotModel::variableMaxLimits() const -> const std::vector<double>&
{
    return m_var_max_limits;
}

auto MoveItRobotModel::variableContinuous() const -> const std::vector<bool>&
{
    return m_var_continuous;
}

/// Set the link for which default forward kinematics is computed.
bool MoveItRobotModel::setPlanningLink(const std::string& name)
{
    if (name.empty()) {
        // clear the planning link
        m_tip_link = nullptr;
        return true;
    }

    if (!m_robot_model->hasLinkModel(name)) {
        ROS_ERROR("planning link '%s' is not in the robot model", name.c_str());
        return false;
    }

    m_tip_link = m_robot_model->getLinkModel(name);
    assert(m_tip_link);
    return true;
}

auto MoveItRobotModel::planningLink() const -> const moveit::core::LinkModel*
{
    return m_tip_link;
}

/// \brief Set the frame the planner accepts kinematics to be computed in.
bool MoveItRobotModel::setPlanningFrame(const std::string& planning_frame)
{
    // TODO: check for frame existence in robot or planning scene?
    m_planning_frame = planning_frame;
    m_planning_frame_is_model_frame = (m_planning_frame == m_robot_model->getModelFrame());
    return true;
}

auto MoveItRobotModel::planningFrame() const -> const std::string&
{
    return m_planning_frame;
}

bool MoveItRobotModel::updateReferenceState(const moveit::core::RobotState& state)
{
    if (state.getRobotModel() != m_robot_model) {
        return false;
    }

    *m_robot_state = state;
    m_robot_state->updateLinkTransforms();
    return true;
}

/// Set the Planning Scene. A planning scene is required if the planning frame
/// differs from the model frame. This function also updates the reference state
/// to match the current state in the planning scene.
bool MoveItRobotModel::setPlanningScene(
    const planning_scene::PlanningSceneConstPtr& scene)
{
    if (!scene) {
        ROS_ERROR("Planning Scene is null");
        return false;
    }

    if (scene->getRobotModel() != m_robot_model) {
        ROS_ERROR("Planning scene is for a different robot model");
        return false;
    }

    if (!updateReferenceState(scene->getCurrentState())) {
        return false;
    }

    m_planning_scene = scene;
    return true;
}

void MoveItRobotModel::printRobotModelInformation()
{
    if (!initialized()) {
        return;
    }

    std::stringstream ss;
    m_joint_group->printGroupInfo(ss);
    ROS_INFO("MoveIt Robot Model for '%s': %s", m_robot_model->getName().c_str(), ss.str().c_str());
}

auto MoveItRobotModel::computeFK(
    const smpl::RobotState& state,
    const std::string& name)
    -> Eigen::Affine3d
{
    assert(initialized() && "MoveItRobotModel is uninitialized");
    assert(state.size() == m_active_var_count && "Incorrect number of joint variables");

    // update all the variables in the robot state
    for (size_t vind = 0; vind < state.size(); ++vind) {
        m_robot_state->setVariablePosition(
                m_active_var_indices[vind], state[vind]);
    }
//    // this would not work in the case of mimic joints, which, for efficiency,
//    // we probaly should not store as part of the state...do these still need
//    // to be updated somehow or are mimic joints automatically updated by the
//    // robot state when we call setJointVariablePosition?
//    m_robot_state->setJointGroupPositions(m_joint_group, state.data());

    m_robot_state->updateLinkTransforms();

    auto T_model_link = m_robot_state->getGlobalLinkTransform(name);

    if (!transformToPlanningFrame(T_model_link)) {
        return Eigen::Affine3d::Identity(); // errors printed within
    }

    return T_model_link; // actually, T_planning_link
}

auto MoveItRobotModel::computeFK(const smpl::RobotState& state)
    -> Eigen::Affine3d
{
    // how do we know what the planning link is for an arbitrary model? This
    // will have to be set from above when a motion plan request comes in with
    // goal constraints for one link

    assert(initialized());
    assert(m_tip_link);
    return computeFK(state, m_tip_link->getName());
}

bool MoveItRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    smpl::RobotState& solution,
    smpl::ik_option::IkOption option)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_ik_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    switch (option) {
    case smpl::ik_option::UNRESTRICTED:
        return computeUnrestrictedIK(pose, start, solution);
    case smpl::ik_option::RESTRICT_XYZ:
        return computeWristIK(pose, start, solution);
    case smpl::ik_option::RESTRICT_RPY:
        return false;
    }

    return false;
}

bool MoveItRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    std::vector<smpl::RobotState>& solutions,
    smpl::ik_option::IkOption option)
{
    // TODO: the indigo version of moveit currently does not support returning
    // multiple ik solutions, so instead we just return the only solution moveit
    // offers; for later versions of moveit, this will need to be implemented
    // properly
    smpl::RobotState solution;
    if (!computeIK(pose, start, solution, option)) {
        return false;
    } else {
        solutions.push_back(std::move(solution));
        return true;
    }
}

auto MoveItRobotModel::redundantVariableCount() const -> const int
{
    return m_redundant_var_count;
}

auto MoveItRobotModel::redundantVariableIndex(int rvidx) const -> const int
{
    return m_redundant_var_indices[rvidx];
}

bool MoveItRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    smpl::RobotState& solution)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_ik_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    return computeUnrestrictedIK(pose, start, solution, m_redundant_ik_group);
}

double MoveItRobotModel::minPosLimit(int vidx) const
{
    return m_var_min_limits[vidx];
}

double MoveItRobotModel::maxPosLimit(int vidx) const
{
    return m_var_max_limits[vidx];
}

bool MoveItRobotModel::hasPosLimit(int vidx) const
{
    return m_var_bounded[vidx];
}

bool MoveItRobotModel::isContinuous(int vidx) const
{
    return m_var_continuous[vidx];
}

double MoveItRobotModel::velLimit(int vidx) const
{
    return m_var_vel_limits[vidx];
}

double MoveItRobotModel::accLimit(int vidx) const
{
    return m_var_acc_limits[vidx];
}

bool MoveItRobotModel::checkJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    assert(state.size() == m_active_var_count);

    for (int vidx = 0; vidx < activeVariableCount(); ++vidx) {
        if (m_var_bounded[vidx]) {
            if ((state[vidx] < m_var_min_limits[vidx]) |
                (state[vidx] > m_var_max_limits[vidx]))
            {
                auto& var_name = planningVariableNames()[vidx];
                ROS_DEBUG("Variable '%s' out of bounds [%0.3f, %0.3f]", var_name.c_str(), m_var_min_limits[vidx], m_var_max_limits[vidx]);
                return false;
            }
        }
    }

    return true;
}

auto MoveItRobotModel::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::RobotModel>() ||
        class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>() ||
        class_code == smpl::GetClassCode<smpl::InverseKinematicsInterface>() ||
        class_code == smpl::GetClassCode<smpl::RedundantManipulatorInterface>())
    {
        return this;
    } else {
        return nullptr;
    }
}

bool MoveItRobotModel::computeUnrestrictedIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    smpl::RobotState& solution,
    bool lock_redundant_joints)
{
    auto T_planning_link = pose;

    // get the transform in the model frame
    if (!transformToModelFrame(T_planning_link)) {
        return false; // errors printed within
    }
    auto& T_model_link = T_planning_link; // rebrand

    ROS_DEBUG_STREAM("start: " << start);
    for (size_t sind = 0; sind < start.size(); ++sind) {
        int avind = m_active_var_indices[sind];
        m_robot_state->setVariablePosition(avind, start[sind]);
    }
    m_robot_state->updateLinkTransforms();

    // The default behavior in KDLKinematicsPlugin is for the first attempt to
    // be seeded with the current state of the joint group and for successive
    // attempts to sample states randomly. A value of 1 here will make the IK
    // solver more deterministic, at the cost of some robustness, which is
    // required for sbpl environments that don't cache successors as they are
    // generated. TODO: does num_attempts have any value for analytical solvers?
    auto num_attempts = 1;
    auto timeout = 0.0;
    moveit::core::GroupStateValidityCallbackFn fn;
    kinematics::KinematicsQueryOptions ops;
    ops.lock_redundant_joints = lock_redundant_joints;
    if (!m_robot_state->setFromIK(
            m_ik_group,
            T_model_link,
            m_tip_link->getName(),
            num_attempts,
            timeout,
            fn,
            ops))
    {
        Eigen::Quaterniond q(T_model_link.rotation());
        ROS_DEBUG("Failed to set from ik to pose (%f, %f, %f, %f, %f, %f, %f)",
                T_model_link.translation().x(),
                T_model_link.translation().y(),
                T_model_link.translation().z(),
                q.x(),
                q.y(),
                q.z(),
                q.w());

        return false;
    }

    // for all revolute joint variables...find the closest 2*pi equivalent that
    // is within bounds
    for (size_t vind = 0; vind < start.size(); ++vind) {
        if (m_var_continuous[vind]) {
            continue;
        }

        int avind = m_active_var_indices[vind];
        double spos = m_robot_state->getVariablePosition(avind);
        double vdiff = start[vind] - spos;
        int twopi_hops = (int)std::fabs(vdiff / (2.0 * M_PI));

        // equivalent within 2*pi of the seed state
        double npos = spos + 2.0 * M_PI * twopi_hops * std::copysign(1.0, vdiff);
        if (fabs(npos - start[vind]) > M_PI) {
            npos += 2.0 * M_PI * std::copysign(1.0, vdiff); // one hop this time
        }

        // set it as the solution
        m_robot_state->setVariablePosition(avind, npos);
        auto* j = m_robot_model->getJointOfVariable(avind);
        if (!m_robot_state->satisfiesBounds(j)) {
            // revert to solution
            m_robot_state->setVariablePosition(avind, spos);
        }

        // TODO: roll extraction into this loop
    }

    if (!m_robot_state->satisfiesBounds(m_joint_group)) {
        ROS_ERROR("KDL Returned invalid joint angles?");
    }

    // extract solution from robot state
    solution.resize(m_active_var_names.size());
    for (size_t avind = 0; avind < m_active_var_names.size(); ++avind) {
        int vind = m_active_var_indices[avind];
        solution[avind] = m_robot_state->getVariablePosition(vind);
    }

    ROS_DEBUG_STREAM("IK Succeeded with solution " << solution);
    return true;
}

bool MoveItRobotModel::computeWristIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    smpl::RobotState& solution)
{
#ifdef PR2_WRIST_IK
    for (size_t sind = 0; sind < start.size(); ++sind) {
        int avind = m_active_var_indices[sind];
        m_robot_state->setVariablePosition(avind, start[sind]);
    }
    m_robot_state->updateLinkTransforms();

    const Eigen::Affine3d& T_model_forearm =
            m_robot_state->getGlobalLinkTransform(m_forearm_roll_link);
    const Eigen::Affine3d& T_model_wrist =
            m_robot_state->getGlobalLinkTransform(m_wrist_roll_link);

    const Eigen::Vector3d forearm_pos(T_model_forearm.translation());
    const Eigen::Vector3d wrist_pos(T_model_wrist.translation());
    const Eigen::Quaterniond forearm_rot(T_model_forearm.rotation());
    const Eigen::Quaterniond wrist_rot(T_model_wrist.rotation());

    // NOTE: switching from fixed axis xyz to fixed axis zyx might screw up
    // the orientation solver but I'm not convinced the orientation solver
    // works anymore
    double fr, fp, fy;
    smpl::angles::get_euler_zyx(forearm_rot, fy, fp, fr);

    double wr, wp, wy;
    smpl::angles::get_euler_zyx(wrist_rot, wy, wp, wr);

    // NOTE: calling fk to get these poses would be more convenient, but may
    // also pollute the robot state being used internally

    const std::vector<double> rpy = { pose[3], pose[4], pose[5] };
    const std::vector<double> forearm_roll_link_pose =
            { forearm_pos.x(), forearm_pos.y(), forearm_pos.z(), fr, fp, fy };
    const std::vector<double> endeff_link_pose =
            { wrist_pos.x(), wrist_pos.y(), wrist_pos.z(), wr, wp, wy };
    const int solnum = 1;
    return m_rpy_solver->computeRPYOnly(
            rpy, start, forearm_roll_link_pose, endeff_link_pose, solnum, solution);
#else
    return false;
#endif
}

bool MoveItRobotModel::transformToPlanningFrame(Eigen::Affine3d& T_model_link) const
{
    if (m_planning_frame_is_model_frame) {
        return true;
    }

    if (!m_planning_scene) {
        ROS_ERROR("Planning Scene required to transform between planning and model frame");
        return false;
    }

    // return the pose of the link in the planning frame
    if (!m_planning_scene->knowsFrameTransform(m_planning_frame) ||
        !m_planning_scene->knowsFrameTransform(m_robot_model->getModelFrame()))
    {
        ROS_ERROR("Planning Scene does not contain transforms to planning frame '%s' or model frame '%s'", m_planning_frame.c_str(), m_robot_model->getModelFrame().c_str());
        return false;
    }

    auto& T_scene_planning = m_planning_scene->getFrameTransform(m_planning_frame);
    auto& T_scene_model = m_planning_scene->getFrameTransform(m_robot_model->getModelFrame());

    T_model_link = T_scene_planning.inverse() * T_scene_model * T_model_link;
    return true;
}

bool MoveItRobotModel::transformToModelFrame(Eigen::Affine3d& T_planning_link) const
{
    if (m_planning_frame_is_model_frame) {
        return true;
    }

    if (!m_planning_scene) {
        ROS_ERROR("Planning Scene required to transform between planning and model frame");
        return false;
    }

    if (!m_planning_scene->knowsFrameTransform(m_planning_frame) ||
        !m_planning_scene->knowsFrameTransform(m_robot_model->getModelFrame()))
    {
        ROS_ERROR("Planning Scene does not contain transforms to planning frame '%s' or model frame '%s'", m_planning_frame.c_str(), m_robot_model->getModelFrame().c_str());
        return false;
    }

    auto& T_scene_planning = m_planning_scene->getFrameTransform(m_planning_frame);
    auto& T_scene_model = m_planning_scene->getFrameTransform(m_robot_model->getModelFrame());

    T_planning_link = T_scene_model.inverse() * T_scene_planning * T_planning_link;
    return true;
}

} // namespace sbpl_interface
