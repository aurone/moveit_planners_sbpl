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

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <sbpl_manipulation_components/robot_model.h>

namespace sbpl_interface {

class MoveItRobotModel : public sbpl_arm_planner::RobotModel
{
public:

    MoveItRobotModel();
    virtual ~MoveItRobotModel();

    bool init(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const std::string& group_name,
        const std::string& planning_frame);

    bool initialized() const;

    virtual bool checkJointLimits(const std::vector<double>& angles);

    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        KDL::Frame& f);

    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        std::vector<double>& pose);

    virtual bool computePlanningLinkFK(
        const std::vector<double>& angles,
        std::vector<double>& pose);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        int option = sbpl_arm_planner::ik_option::UNRESTRICTED);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<std::vector<double> >& solutions,
        int option = sbpl_arm_planner::ik_option::UNRESTRICTED);

    virtual bool computeFastIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);

    virtual void printRobotModelInformation();

    const std::string& planningGroupName() const;
    const moveit::core::JointModelGroup* planningJointGroup() const;

    const std::string& planningFrame() const;

    const moveit::core::LinkModel* planningTipLink() const;

    const std::vector<std::string>& planningVariableNames() const;
    int activeVariableCount() const;
    const std::vector<int>& activeVariableIndices() const;

    const std::vector<double>& variableMinLimits() const;
    const std::vector<double>& variableMaxLimits() const;
    const std::vector<double>& variableIncrements() const;
    const std::vector<bool>& variableContinuous() const;

    moveit::core::RobotModelConstPtr moveitRobotModel() const;

private:

    std::string m_group_name;

    moveit::core::RobotModelConstPtr m_moveit_model;
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

    std::string m_planning_frame;
};

} // namespace sbpl_interface

#endif
