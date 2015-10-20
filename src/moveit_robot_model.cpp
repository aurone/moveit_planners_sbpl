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

#include "moveit_robot_model.h"

#include <eigen_conversions/eigen_kdl.h>
#include <ros/console.h>

#define LEARNING_MOVEIT 1

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
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name)
{
    m_group_name = group_name;
    m_robot_model = robot_model;
    m_robot_state.reset(new moveit::core::RobotState(robot_model));
    m_joint_group = robot_model->getJointModelGroup(group_name);

    // cache the number of active variables in this group
    m_active_var_count = 0;
    std::vector<const moveit::core::JointModel*> active_joints =
        m_joint_group->getActiveJointModels();

    // cache the number of active variables in this group as well as a mapping
    // from each active variable to its index in the robot state
    m_active_var_indices.clear();
    for (const moveit::core::JointModel* joint : active_joints) {
        m_active_var_count += joint->getVariableCount();
        for (size_t vind = 0; vind < joint->getVariableCount(); ++vind) {
            const std::string& var_name = joint->getVariableNames()[vind];
            m_active_var_indices.push_back(robot_model->getVariableIndex(var_name));
        }
    }

    // identify a tip link to use for forward and inverse kinematics
    if (m_joint_group->isChain()) {
        std::vector<const moveit::core::LinkModel*> tips;
        m_joint_group->getEndEffectorTips(tips);
        if (!tips.empty()) {
            m_tip_link = tips.front();
        }
    }

    return true;
}

bool MoveItRobotModel::initialized() const
{
    return (bool)m_joint_group;
}

bool MoveItRobotModel::checkJointLimits(const std::vector<double>& angles)
{
    if (!initialized()) {
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
        return false;
    }

    if (angles.size() != m_active_var_count) {
        ROS_WARN("Incorrect number of joint variables: expected = %d, actual = %zu", m_active_var_count, angles.size());
        return false;
    }

#if LEARNING_MOVEIT
    // update all the variables in the robot state
    for (size_t vind = 0; vind < angles.size(); ++vind) {
        m_robot_state->setVariablePosition(
                m_active_var_indices[vind], angles[vind]);
    }
#else
    // this would not work in the case of mimic joints, which, for efficiency,
    // we probaly should not store as part of the state...do these still need
    // to be updated somehow or are mimic joints automatically updated by the
    // robot state when we call setJointVariablePosition?
    m_robot_state->setJointGroupPositions(m_joint_group, angles.data());
#endif

#if LEARNING_MOVEIT
    m_robot_state->updateLinkTransforms();
#else
#endif

    const Eigen::Affine3d& T_robot_link = m_robot_state->getGlobalLinkTransform(name);
    tf::transformEigenToKDL(T_robot_link, f);
    return true;
}

bool MoveItRobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    std::vector<double>& pose)
{
    if (!initialized()) {
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
        return false;
    }

    if (!m_tip_link || !m_joint_group->canSetStateFromIK(m_tip_link->getName())) {
        ROS_WARN_ONCE("computeIK not available for this Robot Model");
        return false;
    }

    return false;
}

bool MoveItRobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<std::vector<double> >& solutions,
    int option)
{
    if (!initialized()) {
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

} // namespace sbpl_interface
