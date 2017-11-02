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

#include <kdl/frames.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <smpl/robot_model.h>
#include <sbpl_pr2_robot_model/orientation_solver.h>

namespace sbpl_interface {

class MoveItRobotModel :
    public virtual sbpl::motion::RobotModel,
    public virtual sbpl::motion::ForwardKinematicsInterface,
    public virtual sbpl::motion::InverseKinematicsInterface,
    public virtual sbpl::motion::RedundantManipulatorInterface
{
public:

    MoveItRobotModel() = default;

    // disallow copy/assign since RobotModel holds internal references to
    // extension interfaces
    MoveItRobotModel(const MoveItRobotModel&) = delete;
    MoveItRobotModel& operator=(const MoveItRobotModel&) = delete;

    bool init(
        const moveit::core::RobotModelConstPtr& robot_model,
        const std::string& group_name);

    bool initialized() const;

    bool setPlanningLink(const std::string& name);
    const moveit::core::LinkModel* planningTipLink() const;

    void printRobotModelInformation();

    bool setPlanningScene(const planning_scene::PlanningSceneConstPtr& state);

    const std::string& planningGroupName() const;
    const moveit::core::JointModelGroup* planningJointGroup() const;

    bool setPlanningFrame(const std::string& planning_frame);
    const std::string& planningFrame() const;

    /// \name Planning Joint Variable Properties
    ///@{
    int activeVariableCount() const;
    const std::vector<std::string>& planningVariableNames() const;
    const std::vector<int>& activeVariableIndices() const;
    const std::vector<double>& variableMinLimits() const;
    const std::vector<double>& variableMaxLimits() const;
    const std::vector<bool>& variableContinuous() const;
    ///@}

    moveit::core::RobotModelConstPtr moveitRobotModel() const;

    /// \namem Reimplemented Public Functions from Extension
    ///@{
    virtual sbpl::motion::Extension* getExtension(size_t class_code);
    ///@}

    /// \name Reimplemented Public Functions from sbpl::motion::RobotModel
    ///@{
    double minPosLimit(int jidx) const override;
    double maxPosLimit(int jidx) const override;
    bool hasPosLimit(int jidx) const override;
    bool isContinuous(int jidx) const override;
    double velLimit(int jidx) const override;
    double accLimit(int jidx) const override;

    bool checkJointLimits(
        const std::vector<double>& angles,
        bool verbose = false) override;
    ///@}

    /// \name Reimplemented Public Functions from sbpl::motion::ForwardKinematicsInterface
    ///@{
    bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        std::vector<double>& pose);

    bool computePlanningLinkFK(
        const std::vector<double>& angles,
        std::vector<double>& pose);
    ///@}

    /// \name Reimplemented Public Functions from sbpl::motion::InverseKinematicsInterface
    ///@{
    bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        sbpl::motion::ik_option::IkOption option = sbpl::motion::ik_option::UNRESTRICTED) override;

    bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<std::vector<double> >& solutions,
        sbpl::motion::ik_option::IkOption option = sbpl::motion::ik_option::UNRESTRICTED) override;
    ///@}

    /// \name Reimplemented Public Functions from sbpl::motion::RedundantManipulatorInterface
    ///@{
    const int redundantVariableCount() const override;

    const int redundantVariableIndex(int rvidx) const override;

    bool computeFastIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution) override;
    ///@}

private:

    planning_scene::PlanningSceneConstPtr m_planning_scene;

    moveit::core::RobotModelConstPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    std::string m_group_name;
    const moveit::core::JointModelGroup* m_joint_group = nullptr;

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

    std::string m_planning_frame;
    bool m_planning_frame_is_model_frame = false;

    int m_redundant_var_count = 0;
    std::vector<int> m_redundant_var_indices;

    std::shared_ptr<sbpl::motion::RPYSolver> m_rpy_solver;
    std::string m_forearm_roll_link;
    std::string m_wrist_flex_link;
    std::string m_wrist_roll_link;
    std::string m_wrist_flex_joint;

    bool computeUnrestrictedIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        bool lock_redundant_joints = false);

    bool computeWristIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);

    Eigen::Affine3d poseVectorToAffine(const std::vector<double>& pose) const;

    bool transformToPlanningFrame(Eigen::Affine3d& T_model_link) const;
    bool transformToModelFrame(Eigen::Affine3d& T_planning_link) const;
};

} // namespace sbpl_interface

#endif
