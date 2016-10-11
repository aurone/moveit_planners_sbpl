////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#ifndef collision_detection_collision_common_sbpl_h
#define collision_detection_collision_common_sbpl_h

// standard includes
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/allowed_collisions_interface.h>

namespace collision_detection {

struct CollisionGridConfig
{
    double size_x;
    double size_y;
    double size_z;
    double origin_x;
    double origin_y;
    double origin_z;
    double res_m;
    double max_distance_m;
};

// Poorly named struct that represents an efficient pipeline for converting
// RobotStates into RobotCollisionStates for collision checking routines
struct GroupModel
{
    // variables for extracting robot-only state information (state not
    // including joints that connect the robot to the world)
    std::vector<std::string> variable_names;
    std::vector<int> variable_indices;
    bool are_variables_contiguous;
    int variables_offset;

    // variables for mapping joint variables in the order specified by
    // RobotState to the corresponding variables in a RobotCollisionState
    std::vector<int> rcm_joint_indices;

    // robot collision state joint variables for batch processing
    std::vector<double> joint_vars;

    // the final RobotCollisionState
    sbpl::collision::RobotCollisionStatePtr rcs;
};

typedef std::shared_ptr<GroupModel> GroupModelPtr;
typedef std::shared_ptr<const GroupModel> GroupModelConstPtr;

// proxy class to interface with CollisionSpace
class AllowedCollisionMatrixInterface :
    public sbpl::collision::AllowedCollisionsInterface
{
public:

    AllowedCollisionMatrixInterface(const AllowedCollisionMatrix& acm) :
        AllowedCollisionsInterface(),
        m_acm(acm)
    { }

    virtual bool getEntry(
        const std::string& name1,
        const std::string& name2,
        sbpl::collision::AllowedCollision::Type& type) const override
    {
        return m_acm.getEntry(name1, name2, type);
    }

private:

    const AllowedCollisionMatrix& m_acm;
};

void LoadCollisionGridConfig(
    ros::NodeHandle& nh,
    const std::string& param_name,
    CollisionGridConfig& config);

bool LoadJointCollisionGroupMap(
    ros::NodeHandle& nh,
    std::unordered_map<std::string, std::string>& _jcgm_map);

bool ExtractRobotVariables(
    const moveit::core::RobotModel& model,
    std::vector<std::string>& variable_names,
    std::vector<int>& variable_indices,
    bool& are_variables_contiguous,
    int& variables_offset);

bool GetRobotVariableNames(
    const moveit::core::RobotModel& robot_model,
    std::vector<std::string>& var_names,
    std::vector<int>& var_indices);

bool GetRobotCollisionModelJointIndices(
    const std::vector<std::string>& joint_names,
    const sbpl::collision::RobotCollisionModel& rcm,
    std::vector<int>& rcm_joint_indices);

} // namespace collision_detection

#endif
