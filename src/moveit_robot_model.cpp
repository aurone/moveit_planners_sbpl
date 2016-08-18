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

#include <moveit_planners_sbpl/moveit_robot_model.h>

#include <math.h>

#include <eigen_conversions/eigen_kdl.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/utils.h>
#include <tf_conversions/tf_eigen.h>

namespace sbpl_interface {

MoveItRobotModel::MoveItRobotModel() :
    RobotModel(),
    ForwardKinematicsInterface(),
    InverseKinematicsInterface(),
    m_planning_scene(),
    m_robot_model(),
    m_robot_state(),
    m_group_name(),
    m_joint_group(nullptr),
    m_active_var_count(0),
    m_active_var_names(),
    m_active_var_indices(),
    m_var_min_limits(),
    m_var_max_limits(),
    m_var_continuous(),
    m_var_vel_limits(),
    m_var_acc_limits(),
    m_tip_link(nullptr),
    m_planning_frame(),
    m_rpy_solver(),
    m_forearm_roll_link(),
    m_wrist_flex_link(),
    m_wrist_roll_link(),
    m_wrist_flex_joint()
{
}

MoveItRobotModel::~MoveItRobotModel()
{
}

/// \brief Initialize the MoveItRobotModel for the given MoveIt! Robot Model and
///     the group being planned for
bool MoveItRobotModel::init(
    const robot_model::RobotModelConstPtr& robot_model,
    const std::string& group_name)
{
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
    ROS_INFO("Active Variable Count: %d", m_active_var_count);
    ROS_INFO("Active Variable Names: %s", to_string(m_active_var_names).c_str());
    ROS_INFO("Active Variable Indices: %s", to_string(m_active_var_indices).c_str());

    // set the planning joints
    // TODO: should this be joint names or joint variable names?
    std::vector<std::string> planning_joints;
    planning_joints.reserve(m_active_var_count);
    for (const moveit::core::JointModel* joint : active_joints) {
        planning_joints.push_back(joint->getName());
    }
    setPlanningJoints(planning_joints);
    ROS_INFO("Planning Joints: %s", to_string(getPlanningJoints()).c_str());

    // cache the limits and properties of all planning joint variables
    m_var_min_limits.reserve(m_active_var_count);
    m_var_max_limits.reserve(m_active_var_count);
    m_var_continuous.reserve(m_active_var_count);
    m_var_vel_limits.reserve(m_active_var_count);
    m_var_acc_limits.reserve(m_active_var_count);
    for (const std::string& var_name : m_active_var_names) {
        const auto& var_bounds = m_robot_model->getVariableBounds(var_name);
        if (var_bounds.position_bounded_) {
            m_var_continuous.push_back(false);
            m_var_min_limits.push_back(var_bounds.min_position_);
            m_var_max_limits.push_back(var_bounds.max_position_);
        }
        else {
            // slight hack here? !position_bounded_ => continuous?
            m_var_continuous.push_back(true);
            m_var_min_limits.push_back(-M_PI);
            m_var_max_limits.push_back(M_PI);
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

    ROS_INFO("Min Limits: %s", to_string(m_var_min_limits).c_str());
    ROS_INFO("Max Limits: %s", to_string(m_var_max_limits).c_str());
    ROS_INFO("Continuous: %s", to_string(m_var_continuous).c_str());

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
        ROS_INFO("Instantiating orientation solver with limits [%0.3f, %0.3f]", wrist_pitch_min, wrist_pitch_max);
        m_rpy_solver.reset(new sbpl::manip::RPYSolver(wrist_pitch_min, wrist_pitch_max));
    }

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
    return true;
}

sbpl::manip::Extension* MoveItRobotModel::getExtension(size_t class_code)
{
    if (class_code == sbpl::manip::GetClassCode<sbpl::manip::RobotModel>() ||
        class_code == sbpl::manip::GetClassCode<sbpl::manip::ForwardKinematicsInterface>() ||
        class_code == sbpl::manip::GetClassCode<sbpl::manip::InverseKinematicsInterface>() ||
        class_code == sbpl::manip::GetClassCode<sbpl::manip::RedundantManipulatorInterface>())
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
    return !m_var_continuous[jidx];
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
    if (!m_robot_model->hasLinkModel(name)) {
        ROS_ERROR("Cannot set planning link to link that is not in the robot model");
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

bool MoveItRobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    std::vector<double>& pose)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (angles.size() != m_active_var_count) {
        ROS_WARN("Incorrect number of joint variables: expected = %d, actual = %zu", m_active_var_count, angles.size());
        return false;
    }

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

    const Eigen::Affine3d& T_robot_link =
            m_robot_state->getGlobalLinkTransform(name);

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

    Eigen::Affine3d T_planning_link =
            T_scene_planning.inverse() * T_scene_model * T_robot_link;

    // convert to tf for its conversions to euler angles
    tf::Transform tf_planning_link;
    tf::transformEigenToTF(T_planning_link, tf_planning_link);

    double x, y, z, yaw, pitch, roll;
    x = tf_planning_link.getOrigin().x();
    y = tf_planning_link.getOrigin().y();
    z = tf_planning_link.getOrigin().z();
    tf_planning_link.getBasis().getEulerYPR(yaw, pitch, roll);
    pose = { x, y, z, roll, pitch, yaw };
    return true;
}

bool MoveItRobotModel::computePlanningLinkFK(
    const std::vector<double>& angles,
    std::vector<double>& pose)
{
    // how do we know what the planning link is for an arbitrary model? This
    // will have to be set from above when a motion plan request comes in with
    // goal constraints for one link

    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link) {
        return false;
    }
    else {
        return computeFK(angles, m_tip_link->getName(), pose);
    }
}

bool MoveItRobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution,
    sbpl::manip::ik_option::IkOption option)
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
    case sbpl::manip::ik_option::UNRESTRICTED:
        return computeUnrestrictedIK(pose, start, solution);
    case sbpl::manip::ik_option::RESTRICT_XYZ:
        return computeWristIK(pose, start, solution);
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

const moveit::core::JointModelGroup*
MoveItRobotModel::planningJointGroup() const
{
    return m_joint_group;
}

const std::string&
MoveItRobotModel::planningFrame() const
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
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<std::vector<double>>& solutions,
    sbpl::manip::ik_option::IkOption option)
{
    // TODO: the indigo version of moveit currently does not support returning
    // multiple ik solutions, so instead we just return the only solution moveit
    // offers; for later versions of moveit, this will need to be implemented
    // properly
    std::vector<double> solution;
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
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_joint_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    return computeUnrestrictedIK(pose, start, solution, true);
}

bool MoveItRobotModel::computeUnrestrictedIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution,
    bool lock_redundant_joints)
{
    const Eigen::Affine3d T_planning_link = poseVectorToAffine(pose);

    /////////////////////////////////////////////////////////////////
    // Transform the planning frame pose into the kinematics frame //
    /////////////////////////////////////////////////////////////////

    // get the transform in the model frame

    Eigen::Affine3d T_model_link;
    if (!transformToModelFrame(T_planning_link, T_model_link)) {
        ROS_ERROR("Failed to transform pose to the model frame");
        return false;
    }

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
        ROS_DEBUG("IK Failed");
        return false;
    }

    if (!m_robot_state->satisfiesBounds(m_joint_group)) {
        ROS_ERROR("KDL Returned invalid joint angles?");
    }

    // fill in the solution
    solution.resize(m_active_var_names.size());
    for (size_t avind = 0; avind < m_active_var_names.size(); ++avind) {
        int vind = m_active_var_indices[avind];
        solution[avind] = m_robot_state->getVariablePosition(vind);
    }

    ROS_DEBUG("IK Succeeded with solution %s", to_string(solution).c_str());
    return true;
}

bool MoveItRobotModel::computeWristIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution)
{
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

    geometry_msgs::Quaternion fq;
    fq.w = forearm_rot.w();
    fq.x = forearm_rot.x();
    fq.y = forearm_rot.y();
    fq.z = forearm_rot.z();

    geometry_msgs::Quaternion wq;
    wq.w = wrist_rot.w();
    wq.x = wrist_rot.x();
    wq.y = wrist_rot.y();
    wq.z = wrist_rot.z();

    double fr, fp, fy;
    leatherman::getRPY(fq, fr, fp, fy);

    double wr, wp, wy;
    leatherman::getRPY(wq, wr, wp, wy);

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
}

Eigen::Affine3d MoveItRobotModel::poseVectorToAffine(
    const std::vector<double>& pose) const
{
    return Eigen::Translation3d(pose[0], pose[1], pose[2]) *
            Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX());
}

bool MoveItRobotModel::transformToModelFrame(
    const Eigen::Affine3d& T_planning_link,
    Eigen::Affine3d& T_model_link) const
{
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

    T_model_link = T_scene_model.inverse() * T_scene_planning * T_planning_link;
    return true;
}

} // namespace sbpl_interface
