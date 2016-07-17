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

#ifndef collision_detection_CollisionWorldSBPL_h
#define collision_detection_CollisionWorldSBPL_h

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
#include <sbpl_arm_planner/occupancy_grid.h>

// project includes
#include <moveit_planners_sbpl/moveit_robot_model.h>

namespace sbpl_interface {
class MoveItRobotModel;
} // namespace sbpl_interface

namespace collision_detection {

class CollisionWorldSBPL : public CollisionWorld
{
public:

    struct CollisionWorldConfig
    {
        std::string world_frame;
        double size_x;
        double size_y;
        double size_z;
        double origin_x;
        double origin_y;
        double origin_z;
        double res_m;
        double max_distance_m;
    };

    CollisionWorldSBPL();
    CollisionWorldSBPL(const WorldPtr& world);
    CollisionWorldSBPL(const CollisionWorldSBPL& other, const WorldPtr& world);

    virtual ~CollisionWorldSBPL();

    /// \name CollisionWorld API Requirements
    ///@{

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state) const;

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2) const;

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkWorldCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionWorld& other_world) const;

    virtual void checkWorldCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionWorld& other_world,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state) const;

    virtual double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceWorld(const CollisionWorld& world) const;

    virtual double distanceWorld(
        const CollisionWorld& world,
        const AllowedCollisionMatrix& acm) const;

    virtual void setWorld(const WorldPtr& world);

    ///@}

    const distance_field::PropagationDistanceField* distanceField(
        const std::string& group_name) const;

private:

    CollisionWorldConfig m_world_collision_model_config;
    sbpl::collision::CollisionModelConfig m_robot_collision_model_config;

    struct GroupModel
    {
        sbpl::OccupancyGridPtr grid;
        sbpl::collision::CollisionSpacePtr cspace;
    };

    typedef std::shared_ptr<GroupModel> GroupModelPtr;
    typedef std::shared_ptr<const GroupModel> GroupModelConstPtr;

    std::unordered_map<std::string, GroupModelPtr> m_group_models;

    World::ObserverHandle m_observer_handle;

    ros::NodeHandle m_nh;
    ros::Publisher m_cspace_pub;

    // variables describing the robot state, limited to only those joints from
    // the urdf, in the order they are stored within a corresponding RobotState
    bool m_robot_initialized;
    std::vector<std::string> m_robot_variable_names;
    std::vector<int> m_robot_variable_indices;
    bool m_are_robot_variables_contiguous;
    int m_robot_variables_offset;

    void construct();

    std::vector<double> getCheckedVariables(
        const moveit::core::RobotState& state) const;

    // return the internal variable names inside the robot model and their
    // corresponding indices, sorted, within a robot state derived from the
    // model
    bool getRobotVariableNames(
        const moveit::core::RobotModel& model,
        std::vector<std::string>& var_names,
        std::vector<int>& var_indices) const;

    sbpl::collision::CollisionSpacePtr getCollisionSpace(
        const moveit::core::RobotModel& robot_model,
        const std::string& group_name);

    // initialize per-robot information
    void initializeRobotModel(const moveit::core::RobotModel& robot_model);

    void registerWorldCallback();
    void worldUpdate(const World::ObjectConstPtr& object, World::Action action);

    // signals collision in the case of an uninitialized world and no collision
    // in the case of an initialized model with no world; returns true if either
    // of these cases is present and false otherwise
    bool checkDegenerateCollision(CollisionResult& res) const;

    void setVacuousCollision(CollisionResult& res) const;
    void clearAllCollisions(CollisionResult& res) const;

    moveit_msgs::OrientedBoundingBox computeWorldAABB(const World& world) const;
    bool emptyBoundingBox(const moveit_msgs::OrientedBoundingBox& bb) const;

    void addWorldToCollisionSpace(
        const World& world,
        sbpl::collision::CollisionSpace& cspace);

    void updateCollisionSpaceJointState(const moveit::core::RobotState& state);

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

    std::vector<double> extractPlanningVariables(
        const moveit::core::RobotState& state) const;

    void processWorldUpdateUninitialized(const World::ObjectConstPtr& object);
    void processWorldUpdateCreate(const World::ObjectConstPtr& object);
    void processWorldUpdateDestroy(const World::ObjectConstPtr& object);
    void processWorldUpdateMoveShape(const World::ObjectConstPtr& object);
    void processWorldUpdateAddShape(const World::ObjectConstPtr& object);
    void processWorldUpdateRemoveShape(const World::ObjectConstPtr& object);

    // Converts a world object to a collision object.
    // The collision object's frame_id is the planning frame and the operation
    // is unspecified via this call
    bool worldObjectToCollisionObjectMsgFull(
        const World::Object& object,
        moveit_msgs::CollisionObject& collision_object) const;

    bool worldObjectToCollisionObjectMsgName(
        const World::Object& object,
        moveit_msgs::CollisionObject& collision_object) const;
};

} // namespace collision_detection

#endif
