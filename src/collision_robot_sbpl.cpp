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
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state) const
{
    // TODO: implement
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    // TODO: implement
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{

}

double CollisionRobotSBPL::distanceOther(
    const robot_state::RobotState& state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state) const
{
    return std::numeric_limits<double>::quiet_NaN();
}

double CollisionRobotSBPL::distanceOther(
    const robot_state::RobotState& state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state,
    const AllowedCollisionMatrix& acm) const
{
    return std::numeric_limits<double>::quiet_NaN();
}

double CollisionRobotSBPL::distanceSelf(
    const robot_state::RobotState& state) const
{
    return std::numeric_limits<double>::quiet_NaN();
}

double CollisionRobotSBPL::distanceSelf(
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    return std::numeric_limits<double>::quiet_NaN();
}

} // namespace collision_detection
