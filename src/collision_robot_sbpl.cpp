#include "collision_robot_sbpl.h"

namespace collision_detection {

CollisionRobotSBPL::CollisionRobotSBPL(
    const robot_model::RobotModelConstPtr& model,
    double padding,
    double scale)
:
    CollisionRobot(model, padding, scale)
{
    // TODO: implement
}

CollisionRobotSBPL::CollisionRobotSBPL(const CollisionRobotSBPL& other) :
    CollisionRobot(other)
{
    // TODO: implement
}

CollisionRobotSBPL::~CollisionRobotSBPL()
{
}

void CollisionRobotSBPL::checkOtherCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& robot_state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkOtherCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& robot_state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkOtherCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state1,
    const robot_state::RobotState& other_state2) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkOtherCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state1,
    const robot_state::RobotState& other_state2,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    clearAllCollisions(res);
}

double CollisionRobotSBPL::distanceOther(
    const robot_state::RobotState& state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state) const
{
    // TODO: implement
    return -1.0;
}

double CollisionRobotSBPL::distanceOther(
    const robot_state::RobotState& state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    return -1.0;
}

double CollisionRobotSBPL::distanceSelf(
    const robot_state::RobotState& state) const
{
    // TODO: implement
    return -1.0;
}

double CollisionRobotSBPL::distanceSelf(
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    return -1.0;
}

void CollisionRobotSBPL::updatedPaddingOrScaling(
    const std::vector<std::string>& links)
{
    CollisionRobot::updatedPaddingOrScaling(links);
}

void CollisionRobotSBPL::clearAllCollisions(CollisionResult& res) const
{
    res.collision = false;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 100.0;
}

} // namespace collision_detection
