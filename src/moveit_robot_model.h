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
#include <sbpl_manipulation_components/robot_model.h>

namespace sbpl_interface {

class MoveItRobotModel : public sbpl_arm_planner::RobotModel
{
public:

    MoveItRobotModel();
    virtual ~MoveItRobotModel();

    bool init(
        const moveit::core::RobotModelConstPtr& robot_model,
        const std::string& group_name);

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
        int option = 0);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<std::vector<double> >& solutions,
        int option = 0);

    virtual bool computeFastIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);

    virtual void printRobotModelInformation();

private:

    std::string m_group_name;

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
};

} // namespace sbpl_interface

#endif
