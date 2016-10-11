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

#ifndef collision_detection_CollisionRobotSBPL_h
#define collision_detection_CollisionRobotSBPL_h

// system includes
#include <moveit/collision_detection/collision_robot.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/self_collision_model.h>

// module includes
#include "collision_common_sbpl.h"

namespace collision_detection {

class CollisionRobotSBPL : public CollisionRobot
{
public:

    CollisionRobotSBPL(
        const robot_model::RobotModelConstPtr& model,
        double padding = 0.0,
        double scale = 1.0);
    CollisionRobotSBPL(const CollisionRobotSBPL& other);

    virtual ~CollisionRobotSBPL();

    const sbpl::collision::RobotCollisionModelConstPtr&
    robotCollisionModel() const;

    /// \name Reimplemented Public Functions
    ///@{
    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& robot_state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state) const;

    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& robot_state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state1,
        const robot_state::RobotState& other_state2) const;

    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state1,
        const robot_state::RobotState& other_state2,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceOther(
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state) const;

    virtual double distanceOther(
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceSelf(
        const robot_state::RobotState& state) const;

    virtual double distanceSelf(
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;
    ///@}

protected:

    /// \name Reimplemented Protected Functions
    ///@{
    virtual void updatedPaddingOrScaling(const std::vector<std::string>& links);
    ///@}

private:

    typedef std::shared_ptr<const sbpl::collision::CollisionModelConfig>
    CollisionModelConfigConstPtr;

    sbpl::collision::RobotCollisionModelConstPtr m_rcm;

    CollisionGridConfig m_scm_config;

    // robot-only joint variable names
    std::vector<std::string> m_variable_names;

    // ...their indices in the robot state
    std::vector<int> m_variable_indices;

    // whether the indices are contiguous
    bool m_are_variables_contiguous;

    // offset into robot state, if variables are contiguous
    int m_variables_offset;

    // corresponding joint variable indices in robot collision model/state
    std::vector<int> m_rcm_joint_indices;

    // robot collision state joint variables for batch updates
    std::vector<double> m_joint_vars;

    // full collision model state
    sbpl::collision::RobotCollisionStatePtr m_rcs;

    sbpl::OccupancyGridPtr m_grid;
    sbpl::collision::AttachedBodiesCollisionModelPtr m_ab_model;
    sbpl::collision::AttachedBodiesCollisionStatePtr m_ab_state;
    sbpl::collision::SelfCollisionModelPtr m_scm;

    std::unordered_map<std::string, std::string> m_jcgm_map;

    void clearAllCollisions(CollisionResult& res) const;
    void setVacuousCollision(CollisionResult& res) const;

    void checkSelfCollisionMutable(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm);

    double getSelfCollisionPropagationDistance() const;

    sbpl::OccupancyGridPtr createGridFor(
        const CollisionGridConfig& config) const;
};

} // namespace collision_detection

#endif
