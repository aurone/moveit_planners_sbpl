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
#include <sbpl_arm_planner/robot_model.h>
#include <sbpl_pr2_robot_model/orientation_solver.h>

namespace sbpl_interface {

class MoveItRobotModel : public sbpl::manip::RobotModel
{
public:

    MoveItRobotModel();
    virtual ~MoveItRobotModel();

    /// \name sbpl::manip::RobotModel API Requirements
    ///@{

    virtual double minPosLimit(int jidx) const override;
    virtual double maxPosLimit(int jidx) const override;
    virtual bool   hasPosLimit(int jidx) const override;
    virtual double velLimit(int jidx) const override;
    virtual double accLimit(int jidx) const override;

    bool setPlanningLink(const std::string& name) override;

    virtual bool checkJointLimits(
        const std::vector<double>& angles,
        bool verbose = false) override;

    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        std::vector<double>& pose) override;

    virtual bool computePlanningLinkFK(
        const std::vector<double>& angles,
        std::vector<double>& pose);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        sbpl::manip::ik_option::IkOption option = sbpl::manip::ik_option::UNRESTRICTED) override;

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<std::vector<double> >& solutions,
        sbpl::manip::ik_option::IkOption option = sbpl::manip::ik_option::UNRESTRICTED) override;

    virtual bool computeFastIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution) override;

    virtual void printRobotModelInformation() override;

    ///@}

    /// \brief Initialize the MoveItRobotModel for the given MoveIt! Robot Model
    ///     and the group being planned for
    bool init(
        const moveit::core::RobotModelConstPtr& robot_model,
        const std::string& group_name);

    bool initialized() const;

    /// \brief Set the Planning Scene
    ///
    /// The Planning Scene is required, in general, to transform the pose
    /// computed by IK into the planning frame.
    bool setPlanningScene(const planning_scene::PlanningSceneConstPtr& state);

    /// \brief Set the frame the planner accepts kinematics to be computed in
    bool setPlanningFrame(const std::string& planning_frame);

    const std::string& planningGroupName() const;
    const moveit::core::JointModelGroup* planningJointGroup() const;

    const std::string& planningFrame() const;

    const moveit::core::LinkModel* planningTipLink() const;

    /// \brief The names of the variables being planned for
    const std::vector<std::string>& planningVariableNames() const;

    /// \brief The number of variables being planned for
    int activeVariableCount() const;

    /// \brief Return the indices into the moveit::core::RobotState joint
    ///     variable vector of the planning variables
    const std::vector<int>& activeVariableIndices() const;

    /// \name Planning Joint Variable Properties
    ///@{
    const std::vector<double>& variableMinLimits() const;
    const std::vector<double>& variableMaxLimits() const;
    const std::vector<double>& variableIncrements() const;
    const std::vector<bool>& variableContinuous() const;
    ///@}

    moveit::core::RobotModelConstPtr moveitRobotModel() const;

private:

    std::string m_group_name;

    planning_scene::PlanningSceneConstPtr m_planning_scene;
    moveit::core::RobotModelConstPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    // cached pointer to the joint group
    const moveit::core::JointModelGroup* m_joint_group;
    const moveit::core::LinkModel* m_tip_link;

    // number of active joints in this joint group
    int m_active_var_count;

    // map from joints in joint group to their corresponding variable indices
    // in the robot state
    std::vector<int> m_active_var_indices;

    std::vector<std::string> m_active_var_names;

    std::vector<double> m_var_min_limits;
    std::vector<double> m_var_max_limits;
    std::vector<double> m_var_incs;
    std::vector<bool> m_var_continuous;
    std::vector<double> m_var_vel_limits;
    std::vector<double> m_var_acc_limits;

    std::string m_planning_frame;

    std::shared_ptr<sbpl::manip::RPYSolver> m_rpy_solver;
    std::string m_forearm_roll_link;
    std::string m_wrist_flex_link;
    std::string m_wrist_roll_link;
    std::string m_wrist_flex_joint;

    bool computeUnrestrictedIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);

    bool computeWristIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);
};

} // namespace sbpl_interface

#endif
