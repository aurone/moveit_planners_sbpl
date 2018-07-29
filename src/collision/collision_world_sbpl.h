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

#ifndef collision_detection_collision_world_sbpl_h
#define collision_detection_collision_world_sbpl_h

// standard includes
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <moveit/collision_detection/collision_world.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit_msgs/OrientedBoundingBox.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl_collision_checking/world_collision_detector.h>
#include <smpl/occupancy_grid.h>

// project includes
#include "collision_robot_sbpl.h"

namespace collision_detection {

class CollisionWorldSBPL : public CollisionWorld
{
public:

    CollisionWorldSBPL();
    CollisionWorldSBPL(const WorldPtr& world);
    CollisionWorldSBPL(const CollisionWorldSBPL& other, const WorldPtr& world);

    virtual ~CollisionWorldSBPL();

    const smpl::DistanceMapInterface* distanceField(
        const std::string& robot_name,
        const std::string& group_name) const;

    /// \name Reimplemented Public Functions
    ///@{
    void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state) const override;

    void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const override;

    void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2) const override;

    void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const AllowedCollisionMatrix& acm) const override;

    void checkWorldCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionWorld& other_world) const override;

    void checkWorldCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionWorld& other_world,
        const AllowedCollisionMatrix& acm) const override;

    double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state) const;

    double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

    double distanceWorld(const CollisionWorld& world) const;

    double distanceWorld(
        const CollisionWorld& world,
        const AllowedCollisionMatrix& acm) const;

    double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        bool verbose = false) const;

    double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm,
        bool verbose = false) const;

    double distanceWorld(
        const CollisionWorld& world,
        bool verbose = false) const;

    double distanceWorld(
        const CollisionWorld& world,
        const AllowedCollisionMatrix& acm,
        bool verbose = false) const;

    void setWorld(const WorldPtr& world) override;
    ///@}

private:

    CollisionGridConfig m_wcm_config;

    // mapping from joint group name to collision group name
    std::unordered_map<std::string, std::string> m_jcgm_map;

    smpl::OccupancyGridConstPtr m_parent_grid;
    smpl::collision::WorldCollisionModelConstPtr m_parent_wcm;
    smpl::collision::WorldCollisionDetectorConstPtr m_parent_wcd;

    smpl::OccupancyGridPtr m_grid;
    smpl::collision::WorldCollisionModelPtr m_wcm;

    std::unordered_map<std::string, CollisionStateUpdaterPtr> m_updaters;

    World::ObserverHandle m_observer_handle;

    struct ObjectRepPair {
        // originating world object. hold onto it here so we are guaranteed to
        // maintain valid references to shared data e.g. octomap, mesh data,
        // and don't have to make copies.
        World::ObjectConstPtr world_object;

        // CollisionShapes matching world object's shapes
        std::vector<std::unique_ptr<smpl::collision::CollisionShape>> shapes;

        // CollisionObject matching world object
        std::unique_ptr<smpl::collision::CollisionObject> collision_object;
    };

    // TODO: test the semantics of this during copy-on-write
    std::vector<ObjectRepPair> m_collision_objects;

    void construct();

    void copyOnWrite();

    auto FindObjectRepPair(const World::ObjectConstPtr& object)
        -> std::vector<ObjectRepPair>::iterator;

    smpl::OccupancyGridPtr createGridFor(
        const CollisionGridConfig& config) const;

    CollisionStateUpdaterPtr getCollisionStateUpdater(
        const CollisionRobotSBPL& collision_robot,
        const moveit::core::RobotModel& robot_model);

    void registerWorldCallback();
    void worldUpdate(const World::ObjectConstPtr& object, World::Action action);

    void setVacuousCollision(CollisionResult& res) const;
    void clearAllCollisions(CollisionResult& res) const;

    moveit_msgs::OrientedBoundingBox computeWorldAABB(const World& world) const;
    bool emptyBoundingBox(const moveit_msgs::OrientedBoundingBox& bb) const;

    void checkRobotCollisionMutable(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state);

    void checkRobotCollisionMutable(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm);

    void checkRobotCollisionMutable(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2);

    void checkRobotCollisionMutable(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const AllowedCollisionMatrix& acm);

    void processWorldUpdateUninitialized(const World::ObjectConstPtr& object);
    void processWorldUpdateCreate(const World::ObjectConstPtr& object);
    void processWorldUpdateDestroy(const World::ObjectConstPtr& object);
    void processWorldUpdateMoveShape(const World::ObjectConstPtr& object);
    void processWorldUpdateAddShape(const World::ObjectConstPtr& object);
    void processWorldUpdateRemoveShape(const World::ObjectConstPtr& object);

    auto getCollisionRobotVisualization(
        smpl::collision::RobotCollisionState& rcs,
        smpl::collision::AttachedBodiesCollisionState& abcs,
        int gidx) const
        -> visualization_msgs::MarkerArray;
};

} // namespace collision_detection

#endif
