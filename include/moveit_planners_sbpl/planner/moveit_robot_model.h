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

#ifndef sbpl_interface_MoveItRobotModel_h
#define sbpl_interface_MoveItRobotModel_h

// system includes
#include <kdl/frames.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <smpl/robot_model.h>

//#define PR2_WRIST_IK

#ifdef PR2_WRIST_IK
#include <sbpl_pr2_robot_model/orientation_solver.h>
#endif

namespace sbpl_interface {

class MoveItRobotModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface,
    public virtual smpl::InverseKinematicsInterface,
    public virtual smpl::RedundantManipulatorInterface
{
public:

    MoveItRobotModel() = default;

    // disallow copy/assign since RobotModel holds internal references to
    // extension interfaces
    MoveItRobotModel(const MoveItRobotModel&) = delete;
    MoveItRobotModel& operator=(const MoveItRobotModel&) = delete;

    bool init(
        const moveit::core::RobotModelConstPtr& robot_model,
        const std::string& group_name,
        const std::string& ik_group_name = std::string());

    bool initialized() const;

    auto moveitRobotModel() const -> moveit::core::RobotModelConstPtr;
    auto planningGroupName() const -> const std::string&;
    auto planningJointGroup() const -> const moveit::core::JointModelGroup*;

    auto ikGroupName() const -> const std::string&;
    auto ikGroup() const -> const moveit::core::JointModelGroup*;

    int activeVariableCount() const;
    auto planningVariableNames() const -> const std::vector<std::string>&;
    auto activeVariableIndices() const -> const std::vector<int>&;
    auto variableMinLimits() const -> const std::vector<double>&;
    auto variableMaxLimits() const -> const std::vector<double>&;
    auto variableContinuous() const -> const std::vector<bool>&;

    bool setPlanningLink(const std::string& name);
    auto planningLink() const -> const moveit::core::LinkModel*;

    bool setPlanningFrame(const std::string& planning_frame);
    auto planningFrame() const -> const std::string&;

    bool updateReferenceState(const moveit::core::RobotState& state);

    bool setPlanningScene(const planning_scene::PlanningSceneConstPtr& state);

    void printRobotModelInformation();

    auto computeFK(
        const smpl::RobotState& state,
        const std::string& name)
        -> Eigen::Affine3d;

    /// \name ForwardKinematicsInterface Interface
    ///@{
    auto computeFK(const smpl::RobotState& state)
        -> Eigen::Affine3d override;
    ///@}

    /// \name InverseKinematicsInterface Interface
    ///@{
    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        smpl::RobotState& solution,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) override;

    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        std::vector<smpl::RobotState>& solutions,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) override;
    ///@}

    /// \name RedundantManipulatorInterface Interface
    ///@{
    const int redundantVariableCount() const override;

    const int redundantVariableIndex(int rvidx) const override;

    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        smpl::RobotState& solution) override;
    ///@}

    /// \name RobotModel Interface
    ///@{
    double minPosLimit(int jidx) const override;
    double maxPosLimit(int jidx) const override;
    bool hasPosLimit(int jidx) const override;
    bool isContinuous(int jidx) const override;
    double velLimit(int jidx) const override;
    double accLimit(int jidx) const override;

    bool checkJointLimits(
        const smpl::RobotState& state,
        bool verbose = false) override;
    ///@}

    /// \namem Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> smpl::Extension* override;
    ///@}

private:

    moveit::core::RobotModelConstPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    std::string m_group_name;
    std::string m_ik_group_name;
    const moveit::core::JointModelGroup* m_joint_group = nullptr;
    const moveit::core::JointModelGroup* m_ik_group = nullptr;

    int m_active_var_count = 0;
    std::vector<std::string> m_active_var_names;
    std::vector<int> m_active_var_indices; // maps vars to robot state indices

    std::vector<double> m_var_min_limits;
    std::vector<double> m_var_max_limits;
    std::vector<bool> m_var_continuous;
    std::vector<bool> m_var_bounded;
    std::vector<double> m_var_vel_limits;
    std::vector<double> m_var_acc_limits;

    const moveit::core::LinkModel* m_tip_link = nullptr;

    bool m_redundant_ik_group = false;
    int m_redundant_var_count = 0;
    std::vector<int> m_redundant_var_indices;

    std::string m_planning_frame;
    bool m_planning_frame_is_model_frame = false;
    planning_scene::PlanningSceneConstPtr m_planning_scene;

#ifdef PR2_WRIST_IK
    std::unique_ptr<smpl::RPYSolver> m_rpy_solver;
    std::string m_forearm_roll_link;
    std::string m_wrist_flex_link;
    std::string m_wrist_roll_link;
    std::string m_wrist_flex_joint;
#endif

    bool computeUnrestrictedIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        smpl::RobotState& solution,
        bool lock_redundant_joints = false);

    bool computeWristIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        smpl::RobotState& solution);

    bool transformToPlanningFrame(Eigen::Affine3d& T_model_link) const;
    bool transformToModelFrame(Eigen::Affine3d& T_planning_link) const;
};

} // namespace sbpl_interface

#endif
