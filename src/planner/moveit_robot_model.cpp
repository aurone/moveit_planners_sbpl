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

#include <math.h>

#include <eigen_conversions/eigen_kdl.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <tf_conversions/tf_eigen.h>

namespace sbpl_interface {

static const char* LOG = "model";

/// \brief Initialize the MoveItRobotModel for the given MoveIt! Robot Model and
///     the group being planned for
bool MoveItRobotModel::init(
    const robot_model::RobotModelConstPtr& robot_model,
    const std::string& group_name)
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

    m_robot_model = robot_model;
    m_group_name = group_name;
    m_robot_state.reset(new moveit::core::RobotState(robot_model));
    m_robot_state->setToDefaultValues();
    m_joint_group = robot_model->getJointModelGroup(group_name);

    auto active_joints = m_joint_group->getActiveJointModels();

    // cache the number, names, and index within robot state of all active
    // variables in this group
    m_active_var_count = 0;
    m_active_var_names.clear();
    m_active_var_indices.clear();
    for (const moveit::core::JointModel* joint : active_joints) {
        m_active_var_count += joint->getVariableCount();
        for (const std::string& var_name : joint->getVariableNames()) {
            m_active_var_names.push_back(var_name);
            m_active_var_indices.push_back(robot_model->getVariableIndex(var_name));
        }
    }
    ROS_DEBUG_NAMED(LOG, "Active Variable Count: %d", m_active_var_count);
    ROS_DEBUG_NAMED(LOG, "Active Variable Names: %s", to_string(m_active_var_names).c_str());
    ROS_DEBUG_NAMED(LOG, "Active Variable Indices: %s", to_string(m_active_var_indices).c_str());

    // set the planning joints
    // TODO: should this be joint names or joint variable names?
    // -- these should be variable names, but RobotModel will still need to have
    // knowledge of the relationship between joints and variables to support
    // floating joints
    std::vector<std::string> planning_joints;
    planning_joints.reserve(m_active_var_count);
    for (const moveit::core::JointModel* joint : active_joints) {
        planning_joints.insert(
                planning_joints.end(),
                joint->getVariableNames().begin(),
                joint->getVariableNames().end());
    }
    setPlanningJoints(planning_joints);
    ROS_DEBUG_NAMED(LOG, "Planning Joints: %s", to_string(getPlanningJoints()).c_str());

    // cache the limits and properties of all planning joint variables
    m_var_min_limits.reserve(m_active_var_count);
    m_var_max_limits.reserve(m_active_var_count);
    m_var_continuous.reserve(m_active_var_count);
    m_var_vel_limits.reserve(m_active_var_count);
    m_var_acc_limits.reserve(m_active_var_count);
    for (const std::string& var_name : m_active_var_names) {
        const int vidx = m_robot_model->getVariableIndex(var_name);
        const auto& var_bounds = m_robot_model->getVariableBounds(var_name);
        const moveit::core::JointModel* jm =
                m_robot_model->getJointOfVariable(vidx);
        switch (jm->getType()) {
        case moveit::core::JointModel::UNKNOWN:
        case moveit::core::JointModel::JointType::FIXED:
            break; // shouldn't be here
        case moveit::core::JointModel::JointType::REVOLUTE:
            if (var_bounds.position_bounded_) {
                m_var_min_limits.push_back(var_bounds.min_position_);
                m_var_max_limits.push_back(var_bounds.max_position_);
                m_var_continuous.push_back(false);
                m_var_bounded.push_back(true);
            } else {
                // slight hack here? !position_bounded_ => continuous?
                m_var_min_limits.push_back(-M_PI);
                m_var_max_limits.push_back(M_PI);
                m_var_continuous.push_back(true);
                m_var_bounded.push_back(false);
            }
            break;
        case moveit::core::JointModel::JointType::PRISMATIC: {
            if (var_bounds.position_bounded_) {
                m_var_min_limits.push_back(var_bounds.min_position_);
                m_var_max_limits.push_back(var_bounds.max_position_);
                m_var_continuous.push_back(false);
                m_var_bounded.push_back(true);
            } else {
                m_var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                m_var_max_limits.push_back(std::numeric_limits<double>::infinity());
                m_var_continuous.push_back(false);
                m_var_bounded.push_back(false);
            }
        }   break;
        case moveit::core::JointModel::JointType::PLANAR: {
            if (var_bounds.position_bounded_) { // x and y variables
                m_var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                m_var_max_limits.push_back(std::numeric_limits<double>::infinity());
                m_var_continuous.push_back(false);
                m_var_bounded.push_back(false);
            } else { // theta variable
                m_var_min_limits.push_back(-M_PI);
                m_var_max_limits.push_back(M_PI);
                m_var_continuous.push_back(true);
                m_var_bounded.push_back(false);
            }
        }   break;
        case moveit::core::JointModel::JointType::FLOATING: {
            const size_t idx = var_name.find('/');
            if (idx != std::string::npos) {
                std::string local_var_name = var_name.substr(idx + 1);
                if (local_var_name == "trans_x") {
                    m_var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                    m_var_max_limits.push_back(std::numeric_limits<double>::infinity());
                    m_var_continuous.push_back(false);
                    m_var_bounded.push_back(false);
                } else if (local_var_name == "trans_y") {
                    m_var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                    m_var_max_limits.push_back(std::numeric_limits<double>::infinity());
                    m_var_continuous.push_back(false);
                    m_var_bounded.push_back(false);
                } else if (local_var_name == "trans_z") {
                    m_var_min_limits.push_back(-std::numeric_limits<double>::infinity());
                    m_var_max_limits.push_back(std::numeric_limits<double>::infinity());
                    m_var_continuous.push_back(false);
                    m_var_bounded.push_back(false);
                } else if (local_var_name == "rot_w") {
                    m_var_min_limits.push_back(var_bounds.min_position_);
                    m_var_max_limits.push_back(var_bounds.max_position_);
                    m_var_continuous.push_back(false);
                    m_var_bounded.push_back(true);
                } else if (local_var_name == "rot_x") {
                    m_var_min_limits.push_back(var_bounds.min_position_);
                    m_var_max_limits.push_back(var_bounds.max_position_);
                    m_var_continuous.push_back(false);
                    m_var_bounded.push_back(true);
                } else if (local_var_name == "rot_y") {
                    m_var_min_limits.push_back(var_bounds.min_position_);
                    m_var_max_limits.push_back(var_bounds.max_position_);
                    m_var_continuous.push_back(false);
                    m_var_bounded.push_back(true);
                } else if (local_var_name == "rot_z") {
                    m_var_min_limits.push_back(var_bounds.min_position_);
                    m_var_max_limits.push_back(var_bounds.max_position_);
                    m_var_continuous.push_back(false);
                    m_var_bounded.push_back(true);
                } else {
                    ROS_ERROR("Unrecognized variable name '%s'", local_var_name.c_str());
                }
            } else {
                ROS_ERROR("Multi-DOF joint variable name missing '/' separator");
            }
        }   break;
        }

        if (var_bounds.velocity_bounded_) {
            m_var_vel_limits.push_back(var_bounds.max_velocity_);
        }
        else {
            m_var_vel_limits.push_back(0.0);
        }

        if (var_bounds.acceleration_bounded_) {
            m_var_acc_limits.push_back(var_bounds.max_acceleration_);
        }
        else {
            m_var_acc_limits.push_back(0.0);
        }
    }

    ROS_DEBUG_NAMED(LOG, "Min Limits: %s", to_string(m_var_min_limits).c_str());
    ROS_DEBUG_NAMED(LOG, "Max Limits: %s", to_string(m_var_max_limits).c_str());
    ROS_DEBUG_NAMED(LOG, "Continuous: %s", to_string(m_var_continuous).c_str());

    // identify a default tip link to use for forward and inverse kinematics
    // TODO: better default planning link (first tip link)
    if (m_joint_group->isChain()) {
        std::vector<const moveit::core::LinkModel*> tips;
        m_joint_group->getEndEffectorTips(tips);
        for (const moveit::core::LinkModel* tip : tips) {
            if (m_joint_group->canSetStateFromIK(tip->getName())) {
                m_tip_link = tip;
                setPlanningLink(tip->getName());
                break;
            }
        }

        if (!m_tip_link) {
            if (tips.empty()) {
                ROS_WARN_ONCE("No end effector tip link present");
            }
            else {
                ROS_WARN_ONCE("Cannot set state from ik with respect to any available end effector tip links");
            }
        }
    }

    auto solver = m_joint_group->getSolverInstance();
    if (solver) {
        std::vector<unsigned> redundant_joint_indices;
        solver->getRedundantJoints(redundant_joint_indices);
        m_redundant_var_count = (int)redundant_joint_indices.size();
        for (unsigned rvidx : redundant_joint_indices) {
            const std::string& joint_name = solver->getJointNames()[rvidx];
            auto it = std::find(m_active_var_names.begin(), m_active_var_names.end(), joint_name);
            if (it != m_active_var_names.end()) {
                m_redundant_var_indices.push_back(std::distance(m_active_var_names.begin(), it));
            }
            // else hmm...
        }
    }
    else {
        m_redundant_var_count = 0;
        m_redundant_var_indices.clear();
    }

    // TODO: check that we can translate the planning frame to the kinematics
    // frame and vice versa

    // TODO: consider not-readiness if planning scene or planning frame is
    // unspecified

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
        }
        else if (group_name == "left_arm") {
            m_forearm_roll_link = "l_forearm_roll_link";
            m_wrist_flex_link = "l_wrist_flex_link";
            m_wrist_roll_link = "l_wrist_roll_link";
            m_wrist_flex_joint = "l_wrist_flex_joint";
        }
    }

    if (!m_wrist_flex_joint.empty()) {
        const auto& var_bounds =
                m_robot_model->getVariableBounds(m_wrist_flex_joint);

        double wrist_pitch_min = var_bounds.min_position_;
        double wrist_pitch_max = var_bounds.max_position_;
        ROS_DEBUG_NAMED(LOG, "Instantiating orientation solver with limits [%0.3f, %0.3f]", wrist_pitch_min, wrist_pitch_max);
        m_rpy_solver.reset(new sbpl::motion::RPYSolver(wrist_pitch_min, wrist_pitch_max));
    }
#endif

    return true;
}

bool MoveItRobotModel::initialized() const
{
    return (bool)m_joint_group;
}

/// \brief Set the Planning Scene
///
/// The Planning Scene is required, in general, to transform the pose computed
/// by IK into the planning frame.
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

    m_planning_scene = scene;
    *m_robot_state = scene->getCurrentState();
    m_robot_state->updateLinkTransforms();
    return true;
}

/// \brief Set the frame the planner accepts kinematics to be computed in.
bool MoveItRobotModel::setPlanningFrame(const std::string& planning_frame)
{
    // TODO: check for frame existence in robot or planning scene?
    m_planning_frame = planning_frame;
    m_planning_frame_is_model_frame = (m_planning_frame == m_robot_model->getModelFrame());
    return true;
}

sbpl::motion::Extension* MoveItRobotModel::getExtension(size_t class_code)
{
    if (class_code == sbpl::motion::GetClassCode<sbpl::motion::RobotModel>() ||
        class_code == sbpl::motion::GetClassCode<sbpl::motion::ForwardKinematicsInterface>() ||
        class_code == sbpl::motion::GetClassCode<sbpl::motion::InverseKinematicsInterface>() ||
        class_code == sbpl::motion::GetClassCode<sbpl::motion::RedundantManipulatorInterface>())
    {
        return this;
    }
    else {
        return nullptr;
    }
}

double MoveItRobotModel::minPosLimit(int jidx) const
{
    return m_var_min_limits[jidx];
}

double MoveItRobotModel::maxPosLimit(int jidx) const
{
    return m_var_max_limits[jidx];
}

bool MoveItRobotModel::hasPosLimit(int jidx) const
{
    return m_var_bounded[jidx];
}

bool MoveItRobotModel::isContinuous(int jidx) const
{
    return m_var_continuous[jidx];
}

double MoveItRobotModel::velLimit(int jidx) const
{
    return m_var_vel_limits[jidx];
}

double MoveItRobotModel::accLimit(int jidx) const
{
    return m_var_acc_limits[jidx];
}

/// \brief Set the link for which default forward kinematics is computed.
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

bool MoveItRobotModel::checkJointLimits(
    const std::vector<double>& angles,
    bool verbose)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (angles.size() != m_active_var_count) {
        ROS_WARN("Incorrect number of joint variables: expected = %d, actual = %zu", m_active_var_count, angles.size());
        return false;
    }

    for (int vidx = 0; vidx < activeVariableCount(); ++vidx) {
        const std::string& var_name = planningVariableNames()[vidx];
        const auto& bounds = m_robot_model->getVariableBounds(var_name);
        if (bounds.position_bounded_) {
            if (angles[vidx] < bounds.min_position_ ||
                angles[vidx] > bounds.max_position_)
            {
                ROS_DEBUG("Variable '%s' out of bounds [%0.3f, %0.3f]", var_name.c_str(), bounds.min_position_, bounds.max_position_);
                return false;
            }
        }
    }

    return true;

    // TODO: why must the joint values for continuous joints be in the range [-pi, pi]
//    return m_joint_group->satisfiesPositionBounds(angles_copy.data());
}

Eigen::Affine3d MoveItRobotModel::computeFK(
    const sbpl::motion::RobotState& angles,
    const std::string& name)
{
    assert(initialized() && "MoveItRobotModel is uninitialized");
    assert(angles.size() == m_active_var_count && "Incorrect number of joint variables");

    // update all the variables in the robot state
    for (size_t vind = 0; vind < angles.size(); ++vind) {
        m_robot_state->setVariablePosition(
                m_active_var_indices[vind], angles[vind]);
    }
//    // this would not work in the case of mimic joints, which, for efficiency,
//    // we probaly should not store as part of the state...do these still need
//    // to be updated somehow or are mimic joints automatically updated by the
//    // robot state when we call setJointVariablePosition?
//    m_robot_state->setJointGroupPositions(m_joint_group, angles.data());

    m_robot_state->updateLinkTransforms();

    auto T_model_link = m_robot_state->getGlobalLinkTransform(name);

    if (!transformToPlanningFrame(T_model_link)) {
        return Eigen::Affine3d::Identity(); // errors printed within
    }

    const auto& T_planning_link = T_model_link; // rebrand
    return T_planning_link;
}

auto MoveItRobotModel::computeFK(const sbpl::motion::RobotState& angles)
    -> Eigen::Affine3d
{
    // how do we know what the planning link is for an arbitrary model? This
    // will have to be set from above when a motion plan request comes in with
    // goal constraints for one link

    assert(initialized());
    assert(m_tip_link);
    return computeFK(angles, m_tip_link->getName());
}

bool MoveItRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const sbpl::motion::RobotState& start,
    sbpl::motion::RobotState& solution,
    sbpl::motion::ik_option::IkOption option)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_joint_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    switch (option) {
    case sbpl::motion::ik_option::UNRESTRICTED:
        return computeUnrestrictedIK(pose, start, solution);
    case sbpl::motion::ik_option::RESTRICT_XYZ:
        return computeWristIK(pose, start, solution);
    case sbpl::motion::ik_option::RESTRICT_RPY:
        return false;
    }

    return false;
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

const std::string& MoveItRobotModel::planningGroupName() const
{
    return m_group_name;
}

auto MoveItRobotModel::planningJointGroup() const
    -> const moveit::core::JointModelGroup*
{
    return m_joint_group;
}

const std::string& MoveItRobotModel::planningFrame() const
{
    return m_planning_frame;
}

const moveit::core::LinkModel* MoveItRobotModel::planningTipLink() const
{
    return m_tip_link;
}

/// \brief Return the names of the variables being planned for.
const std::vector<std::string>& MoveItRobotModel::planningVariableNames() const
{
    return m_active_var_names;
}

/// \brief Return the number of variables being planned for.
int MoveItRobotModel::activeVariableCount() const
{
    return m_active_var_count;
}

/// \brief Return the indices into the moveit::core::RobotState joint
///     variable vector of the planning variables.
const std::vector<int>& MoveItRobotModel::activeVariableIndices() const
{
    return m_active_var_indices;
}

const std::vector<double>& MoveItRobotModel::variableMinLimits() const
{
    return m_var_min_limits;
}

const std::vector<double>& MoveItRobotModel::variableMaxLimits() const
{
    return m_var_max_limits;
}

const std::vector<bool>& MoveItRobotModel::variableContinuous() const
{
    return m_var_continuous;
}

moveit::core::RobotModelConstPtr MoveItRobotModel::moveitRobotModel() const
{
    return m_robot_model;
}

bool MoveItRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const sbpl::motion::RobotState& start,
    std::vector<sbpl::motion::RobotState>& solutions,
    sbpl::motion::ik_option::IkOption option)
{
    // TODO: the indigo version of moveit currently does not support returning
    // multiple ik solutions, so instead we just return the only solution moveit
    // offers; for later versions of moveit, this will need to be implemented
    // properly
    sbpl::motion::RobotState solution;
    if (!computeIK(pose, start, solution, option)) {
        return false;
    }
    else {
        solutions.push_back(std::move(solution));
        return true;
    }
}

const int MoveItRobotModel::redundantVariableCount() const
{
    return m_redundant_var_count;
}

const int MoveItRobotModel::redundantVariableIndex(int rvidx) const
{
    return m_redundant_var_indices[rvidx];
}

bool MoveItRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const sbpl::motion::RobotState& start,
    sbpl::motion::RobotState& solution)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_joint_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    bool lock_redundant_joints = m_redundant_var_count > 0; // because i got a crash once from this?
    return computeUnrestrictedIK(pose, start, solution, lock_redundant_joints);
}

bool MoveItRobotModel::computeUnrestrictedIK(
    const Eigen::Affine3d& pose,
    const sbpl::motion::RobotState& start,
    sbpl::motion::RobotState& solution,
    bool lock_redundant_joints)
{
    auto T_planning_link = pose;

    // get the transform in the model frame
    if (!transformToModelFrame(T_planning_link)) {
        return false; // errors printed within
    }
    const auto& T_model_link = T_planning_link; // rebrand

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
    const int num_attempts = 1;
    const double timeout = 0.0;
    moveit::core::GroupStateValidityCallbackFn fn;
    kinematics::KinematicsQueryOptions ops;
    ops.lock_redundant_joints = lock_redundant_joints;
    if (!m_robot_state->setFromIK(
            m_joint_group,
            T_model_link, m_tip_link->getName(),
            num_attempts, timeout, fn, ops))
    {
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
        const moveit::core::JointModel* j = m_robot_model->getJointOfVariable(avind);
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

    ROS_DEBUG("IK Succeeded with solution %s", to_string(solution).c_str());
    return true;
}

bool MoveItRobotModel::computeWristIK(
    const Eigen::Affine3d& pose,
    const sbpl::motion::RobotState& start,
    sbpl::motion::RobotState& solution)
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
    sbpl::angles::get_euler_zyx(forearm_rot, fy, fp, fr);

    double wr, wp, wy;
    sbpl::angles::get_euler_zyx(wrist_rot, wy, wp, wr);

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

    const Eigen::Affine3d& T_scene_planning =
            m_planning_scene->getFrameTransform(m_planning_frame);
    const Eigen::Affine3d& T_scene_model =
            m_planning_scene->getFrameTransform(m_robot_model->getModelFrame());

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

    const Eigen::Affine3d& T_scene_planning =
            m_planning_scene->getFrameTransform(m_planning_frame);
    const Eigen::Affine3d& T_scene_model =
            m_planning_scene->getFrameTransform(m_robot_model->getModelFrame());

    T_planning_link = T_scene_model.inverse() * T_scene_planning * T_planning_link;
    return true;
}

} // namespace sbpl_interface
