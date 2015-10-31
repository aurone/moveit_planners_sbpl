#include "collision_world_sbpl.h"

#include <ros/ros.h>

namespace collision_detection {

CollisionWorldSBPL::CollisionWorldSBPL() :
    CollisionWorld(),
    m_dfield(),
    m_grid(),
    m_cspace(),
    m_observer_handle()
{
    // TODO: implement
    ROS_INFO("CollisionWorldSBPL()");
}

CollisionWorldSBPL::CollisionWorldSBPL(const WorldPtr& world) :
    CollisionWorld(world),
    m_dfield(),
    m_grid(),
    m_cspace(),
    m_observer_handle()
{
    // TODO: implement
    ROS_INFO("CollisionWorldSBPL(const WorldPtr&)");

    registerWorldCallback();
}

CollisionWorldSBPL::CollisionWorldSBPL(
    const CollisionWorldSBPL& other,
    const WorldPtr& world)
:
    CollisionWorld(other, world),
    m_dfield(),
    m_grid(),
    m_cspace(),
    m_observer_handle()
{
    // TODO: implement
    ROS_INFO("CollisionWorldSBPL(CollisionWorldSBPL&, const WorldPtr&)");
    registerWorldCallback();
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
    if (checkDegenerateCollision(res)) {
        return;
    }
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    if (checkDegenerateCollision(res)) {
        return;
    }
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    if (checkDegenerateCollision(res)) {
        return;
    }
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    if (checkDegenerateCollision(res)) {
        return;
    }
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world) const
{
    if (checkDegenerateCollision(res)) {
        return;
    }
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world,
    const AllowedCollisionMatrix& acm) const
{
    if (checkDegenerateCollision(res)) {
        return;
    }
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    return -1.0;
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(const CollisionWorld& world) const
{
    // TODO: implement
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(
    const CollisionWorld& world,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    return -1.0;
}

void CollisionWorldSBPL::setWorld(const WorldPtr& world)
{
    // deregister update callback (we should always have a callback registered
    // if we have a world)
    const WorldPtr& curr_world = getWorld();
    if (curr_world) {
        curr_world->removeObserver(m_observer_handle);
    }

    CollisionWorld::setWorld(world);
    ROS_INFO("setWorld(const WorldPtr&)");

    registerWorldCallback();
}

bool CollisionWorldSBPL::init(
    const std::string& urdf_string,
    const std::string& group_name,
    const sbpl::collision::CollisionModelConfig& config)
{
    ROS_INFO("CollisionWorldSBPL::init");
    if (!getWorld()) {
        ROS_WARN("Collision World SBPL requires a World before initalization");
        return false;
    }
}

bool CollisionWorldSBPL::initialized() const
{
    // if no world, collisions are vacuously empty, otherwise initialization is
    // required to determine collisions
    return (!getWorld() || m_cspace);
}

void CollisionWorldSBPL::registerWorldCallback()
{
    auto ocfn = boost::bind(&CollisionWorldSBPL::worldUpdate, this, _1, _2);
    m_observer_handle = getWorld()->addObserver(ocfn);
}

void CollisionWorldSBPL::worldUpdate(
    const World::ObjectConstPtr& object,
    World::Action action)
{
    ROS_INFO("CollisionWorldSBPL::worldUpdate()");
}

bool CollisionWorldSBPL::checkDegenerateCollision(CollisionResult& res) const
{
    if (!initialized()) {
        setVacuousCollision(res);
        return true;
    }

    if (!getWorld()) {
        clearAllCollisions(res);
        return true;
    }

    return false;
}

void CollisionWorldSBPL::setVacuousCollision(CollisionResult& res) const
{
    res.collision = true;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 0.0;
}

void CollisionWorldSBPL::clearAllCollisions(CollisionResult& res) const
{
    res.collision = false;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 100.0;
}

} // collision_detection
