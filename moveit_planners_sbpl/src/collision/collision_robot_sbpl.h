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
#include <smpl/occupancy_grid.h>
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/self_collision_model.h>
#include <smpl/forward.h>

// module includes
#include "collision_common_sbpl.h"
#include "config.h"

namespace smpl {
SBPL_CLASS_FORWARD(OccupancyGrid);
}

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

    auto robotCollisionModel() const
        -> const smpl::collision::RobotCollisionModelConstPtr&;

    auto robotMotionCollisionModel() const
        -> const smpl::collision::RobotMotionCollisionModelConstPtr&;

    /// \name CollisionRobot Interface
    ///@{
    void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state) const override;

    void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const override;

    void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2) const override;

    void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const AllowedCollisionMatrix& acm) const override;

    void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state) const override;

    void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state,
        const AllowedCollisionMatrix& acm) const override;

    void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state1,
        const robot_state::RobotState& other_state2) const override;

    void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state1,
        const robot_state::RobotState& other_state2,
        const AllowedCollisionMatrix& acm) const override;

#if COLLISION_DETECTION_SBPL_ROS_VERSION == COLLISION_DETECTION_SBPL_ROS_KINETIC

    void distanceSelf(
        const DistanceRequest& req,
        DistanceResult& res,
        const robot_state::RobotState& state) const override;

    void distanceOther(
        const DistanceRequest& req,
        DistanceResult& res,
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state) const override;

#else

    double distanceSelf(const robot_state::RobotState& state) const override;

    double distanceSelf(
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const override;

    double distanceOther(
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state) const override;

    double distanceOther(
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state,
        const AllowedCollisionMatrix& acm) const override;
#endif

    ///@}

protected:

    /// \name Reimplemented Protected Functions
    ///@{
    void updatedPaddingOrScaling(const std::vector<std::string>& links) override;
    ///@}

private:

    typedef std::shared_ptr<const smpl::collision::CollisionModelConfig>
    CollisionModelConfigConstPtr;

    CollisionGridConfig m_scm_config;

    // mapping from joint group name to collision group name
    std::unordered_map<std::string, std::string> m_jcgm_map;

    smpl::collision::RobotCollisionModelConstPtr m_rcm;
    smpl::collision::RobotMotionCollisionModelConstPtr m_rmcm;

    CollisionStateUpdater m_updater;

    // self colllision models
    smpl::OccupancyGridPtr m_grid;
    smpl::collision::SelfCollisionModelPtr m_scm;

    void setVacuousCollision(CollisionResult& res) const;

    void checkSelfCollisionMutable(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm);

    void checkSelfCollisionMutable(
        const CollisionRequest& req,
        CollisionResult& res,
        const moveit::core::RobotState& state1,
        const moveit::core::RobotState& state2,
        const AllowedCollisionMatrix& acm);

    bool updateAttachedBodies(const moveit::core::RobotState& state);

    double getSelfCollisionPropagationDistance() const;

    smpl::OccupancyGridPtr createGridFor(
        const CollisionGridConfig& config) const;

    visualization_msgs::MarkerArray
    getCollisionRobotVisualization(
        smpl::collision::RobotCollisionState& rcs,
        smpl::collision::AttachedBodiesCollisionState& abcs,
        int gidx) const;
};

} // namespace collision_detection

#endif
