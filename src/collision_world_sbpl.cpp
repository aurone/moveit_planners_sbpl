#include "collision_world_sbpl.h"

namespace collision_detection {

CollisionWorldSBPL::CollisionWorldSBPL() :
    CollisionWorld()
{
    // TODO: implement
}

CollisionWorldSBPL::CollisionWorldSBPL(const WorldPtr& world) :
    CollisionWorld(world)
{
    // TODO: implement
}

CollisionWorldSBPL::CollisionWorldSBPL(
    const CollisionWorldSBPL& other,
    const WorldPtr& world)
:
    CollisionWorld(other, world)
{
    // TODO: implement
}

CollisionWorldSBPL::~CollisionWorldSBPL()
{
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state) const
{
    // TODO: implement
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    const CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    // TODO: implement
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    const CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    const CollisionResult& res,
    const CollisionWorld& other_world) const
{
    // TODO: implement
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    const CollisionResult& res,
    const CollisionWorld& other_world,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
}

} // collision_detection
