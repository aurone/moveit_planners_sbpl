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

namespace sbpl_interface {

MoveItRobotModel::MoveItRobotModel() :
    m_group_name(),
    m_robot_model(),
    m_robot_state(),
    m_joint_group(nullptr),
    m_tip_link(nullptr),
    m_active_var_count(0),
    m_active_var_indices()
{
}

MoveItRobotModel::~MoveItRobotModel()
{

}

bool MoveItRobotModel::init(
    const robot_model::RobotModelConstPtr& robot_model,
    const std::string& group_name)
{
    m_robot_model = robot_model;
    m_group_name = group_name;
    m_robot_state.reset(new moveit::core::RobotState(robot_model));
    m_robot_state->setToDefaultValues();
    m_joint_group = robot_model->getJointModelGroup(group_name);

    // cache the number of active variables in this group
    m_active_var_count = 0;
    std::vector<const moveit::core::JointModel*> active_joints =
        m_joint_group->getActiveJointModels();

    // cache the number of active variables in this group as well as a mapping
    // from each active variable to its index in the robot state
    m_active_var_indices.clear();
    for (size_t jind = 0; jind < active_joints.size(); ++jind) {
        const moveit::core::JointModel* joint = active_joints[jind];
        m_active_var_count += joint->getVariableCount();
        for (size_t vind = 0; vind < joint->getVariableCount(); ++vind) {
            const std::string& var_name = joint->getVariableNames()[vind];
            m_active_var_names.push_back(var_name);
            m_active_var_indices.push_back(robot_model->getVariableIndex(var_name));
        }
    }
    ROS_INFO("Active Variable Names: %s", to_string(m_active_var_names).c_str());
    ROS_INFO("Active Variable Indices: %s", to_string(m_active_var_indices).c_str());

    // cache the names of all planning joint variables
    std::vector<std::string> planning_joints;
    planning_joints.reserve(m_active_var_count);
    for (size_t jind = 0; jind < active_joints.size(); ++jind) {
        const moveit::core::JointModel* joint = active_joints[jind];
        planning_joints.push_back(joint->getName());
    }
    setPlanningJoints(planning_joints);
    ROS_INFO("Planning Joints: %s", to_string(getPlanningJoints()).c_str());

    // cache the limits and properties of all planning joint variables
    m_var_min_limits.reserve(m_active_var_count);
    m_var_max_limits.reserve(m_active_var_count);
    m_var_incs.reserve(m_active_var_count);
    m_var_continuous.reserve(m_active_var_count);
    for (size_t jind = 0; jind < active_joints.size(); ++jind) {
        const moveit::core::JointModel* joint = active_joints[jind];
        for (size_t vind = 0; vind < joint->getVariableCount(); ++vind) {
            const std::string& var_name = joint->getVariableNames()[vind];
            const moveit::core::VariableBounds& var_bounds =
                    joint->getVariableBounds(var_name);
            if (var_bounds.position_bounded_) {
                m_var_continuous.push_back(false);
                m_var_min_limits.push_back(var_bounds.min_position_);
                m_var_max_limits.push_back(var_bounds.max_position_);
                m_var_incs.push_back(sbpl::utils::ToRadians(1.0));
            }
            else {
                // slight hack here? !position_bounded_ => continuous?
                m_var_continuous.push_back(true);
                m_var_min_limits.push_back(-M_PI);
                m_var_max_limits.push_back(M_PI);
                m_var_incs.push_back(sbpl::utils::ToRadians(1.0));
            }
        }
    }

    ROS_INFO("Min Limits: %s", to_string(m_var_min_limits).c_str());
    ROS_INFO("Max Limits: %s", to_string(m_var_max_limits).c_str());
    ROS_INFO("Continuous: %s", to_string(m_var_continuous).c_str());
    ROS_INFO("Increments: %s", to_string(m_var_incs).c_str());

    // identify a tip link to use for forward and inverse kinematics
    if (m_joint_group->isChain()) {
        std::vector<const moveit::core::LinkModel*> tips;
        m_joint_group->getEndEffectorTips(tips);
        if (!tips.empty()) {
            m_tip_link = tips.front();
            setPlanningLink(m_tip_link->getName());
        }
    }

    // identify the frame to be used for planning
    const moveit::core::JointModel* root_joint = m_joint_group->getCommonRoot();
    if (!root_joint) {
        ROS_WARN("No common root exists for joint group '%s'", group_name.c_str());
        return false;
    }

    // TODO: check that we can translate the planning frame to the kinematics
    // frame and vice versa

    // TODO: consider not-readiness if planning scene or planning frame is
    // unspecified

    return true;
}

bool MoveItRobotModel::initialized() const
{
    return (bool)m_joint_group;
}

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

bool MoveItRobotModel::setPlanningFrame(const std::string& planning_frame)
{
    // TODO: check for frame existence in robot or planning scene?
    m_planning_frame = planning_frame;
    return true;
}

bool MoveItRobotModel::checkJointLimits(const std::vector<double>& angles)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (angles.size() != m_active_var_count) {
        ROS_WARN("Incorrect number of joint variables: expected = %d, actual = %zu", m_active_var_count, angles.size());
        return false;
    }

    return m_joint_group->satisfiesPositionBounds(angles.data());
}

bool MoveItRobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    KDL::Frame& f)
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

//    if (!m_robot_state->setFromIK(m_joint_group, T_robot_link)) {
//        ROS_ERROR("Failed to compute IK?");
//    }

    const Eigen::Affine3d& T_scene_planning =
            m_planning_scene->getFrameTransform(m_planning_frame);
    const Eigen::Affine3d& T_scene_model =
            m_planning_scene->getFrameTransform(m_robot_model->getModelFrame());

    Eigen::Affine3d T_planning_link =
            T_scene_planning.inverse() * T_scene_model * T_robot_link;

    tf::transformEigenToKDL(T_planning_link, f);
    return true;
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

    KDL::Frame f;
    if (!computeFK(angles, name, f)) {
        return false;
    }

    double roll, pitch, yaw;
    f.M.GetRPY(roll, pitch, yaw);

    pose = { f.p.x(), f.p.y(), f.p.z(), roll, pitch, yaw };
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
    int option)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_joint_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    Eigen::Affine3d T_planning_link;
//    T_planning_link =
//            Eigen::Translation3d(pose[0], pose[1], pose[2]) *
//            Eigen::AngleAxisd(pose[3], Eigen::Vector3d(0.0, 0.0, 1.0)) *
//            Eigen::AngleAxisd(pose[4], Eigen::Vector3d(0.0, 1.0, 0.0)) *
//            Eigen::AngleAxisd(pose[5], Eigen::Vector3d(1.0, 0.0, 0.0));
    T_planning_link =
            Eigen::Translation3d(pose[0], pose[1], pose[2]) *
            Eigen::AngleAxisd(pose[5], Eigen::Vector3d(0.0, 0.0, 1.0)) *
            Eigen::AngleAxisd(pose[4], Eigen::Vector3d(0.0, 1.0, 0.0)) *
            Eigen::AngleAxisd(pose[3], Eigen::Vector3d(1.0, 0.0, 0.0));

    // get the transform in the model frame

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

    Eigen::Affine3d T_model_link =
            T_scene_model.inverse() *
            T_scene_planning *
            T_planning_link;

    Eigen::Affine3d T_ik_link = T_model_link;
    m_robot_state->setToIKSolverFrame(T_ik_link, m_joint_group->getSolverInstance());

    for (size_t sind = 0; sind < start.size(); ++sind) {
        int avind = m_active_var_indices[sind];
        m_robot_state->setVariablePosition(avind, start[sind]);
    }
    m_robot_state->updateLinkTransforms();

    if (m_robot_state->setFromIK(m_joint_group, T_model_link, 10, 0.1)) {
        solution.resize(m_active_var_names.size());
        for (size_t avind = 0; avind < m_active_var_names.size(); ++avind) {
            int vind = m_active_var_indices[avind];
            solution[avind] = m_robot_state->getVariablePosition(vind);
        }
        return true;
    }
    else {
        Eigen::Vector3d ik_pos(T_model_link.translation());
        Eigen::Quaterniond ik_rot(T_model_link.rotation());
        geometry_msgs::Quaternion q;
        q.w = ik_rot.w();
        q.x = ik_rot.x();
        q.y = ik_rot.y();
        q.z = ik_rot.z();
        double r, p, y;
        leatherman::getRPY(q, r, p, y);
        ROS_ERROR("Failed to compute IK to pose (%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f) in %s for joint group '%s'",
                ik_pos.x(), ik_pos.y(), ik_pos.z(), r, p, y, m_robot_model->getModelFrame().c_str(), m_group_name.c_str());

        return false;
    }
}

bool MoveItRobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<std::vector<double>>& solutions,
    int option)
{
    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_joint_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    return false;
}

bool MoveItRobotModel::computeFastIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution)
{
    // TODO: find out what this method is supposed to accomplish over the other
    // methods...

    if (!initialized()) {
        ROS_ERROR("MoveIt! Robot Model is uninitialized");
        return false;
    }

    if (!m_tip_link || !m_joint_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    return computeIK(
            pose, start, solution, sbpl_arm_planner::ik_option::UNRESTRICTED);
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

const std::vector<std::string>& MoveItRobotModel::planningVariableNames() const
{
    return m_active_var_names;
}

int MoveItRobotModel::activeVariableCount() const
{
    return m_active_var_count;
}

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

const std::vector<double>& MoveItRobotModel::variableIncrements() const
{
    return m_var_incs;
}

const std::vector<bool>& MoveItRobotModel::variableContinuous() const
{
    return m_var_continuous;
}

moveit::core::RobotModelConstPtr MoveItRobotModel::moveitRobotModel() const
{
    return m_robot_model;
}

} // namespace sbpl_interface
