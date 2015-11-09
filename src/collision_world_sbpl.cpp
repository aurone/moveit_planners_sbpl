#include "collision_world_sbpl.h"

#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>

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

    clearAllCollisions(res);
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

    clearAllCollisions(res);
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

    clearAllCollisions(res);
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

    clearAllCollisions(res);
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world) const
{
    if (checkDegenerateCollision(res)) {
        return;
    }

    clearAllCollisions(res);
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

    clearAllCollisions(res);
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
    if (!m_urdf_string.empty()) {
        ROS_WARN("Collision World SBPL cannot be reinitialized");
        return false;
    }

    ROS_INFO("CollisionWorldSBPL::init");
    WorldPtr curr_world = getWorld();
    if (!curr_world) {
        ROS_WARN("Collision World SBPL requires a World before initalization");
        return false;
    }

    // create distance field large enough to hold world
    moveit_msgs::OrientedBoundingBox world_bb = computeWorldAABB(*curr_world);

    // TODO: being outside the distance field considered valid? it probably
    // should be since there's no notion of a bounding box in the world. If
    // boundaries are considered part of the collision checking step, then
    // they should probably be expressed as part of the workspace message rather
    // than here

    const double df_max_distance = 0.2; // TODO: dependent on collision model

    // TODO: pad the bounding box by the max propagation distance?
    world_bb.extents.x += 2 * df_max_distance;
    world_bb.extents.y += 2 * df_max_distance;
    world_bb.extents.z += 2 * df_max_distance;

    m_cspace.reset();
    m_grid.reset();
    m_dfield.reset();

    const double df_size_x = world_bb.extents.x;
    const double df_size_y = world_bb.extents.y;
    const double df_size_z = world_bb.extents.z;
    const double df_res_m = 0.02; // TODO: non-hardcode
    const double df_origin_x = world_bb.pose.position.x;
    const double df_origin_y = world_bb.pose.position.y;
    const double df_origin_z = world_bb.pose.position.z;

    ROS_INFO("Initializing distance field to size (%0.3f, %0.3f, %0.3f) at (%0.3f, %0.3f, %0.3f)", df_size_x, df_size_y, df_size_z, df_origin_x, df_origin_y, df_origin_z);
    m_dfield.reset(new distance_field::PropagationDistanceField(
            df_size_x, df_size_y, df_size_z,
            df_res_m,
            df_origin_x, df_origin_y, df_origin_z,
            df_max_distance,
            false));
    m_grid.reset(new sbpl_arm_planner::OccupancyGrid(m_dfield.get()));
    m_cspace.reset(new sbpl::collision::SBPLCollisionSpace(m_grid.get()));

    if (m_cspace->init(urdf_string, group_name, config)) {
        // save these parameters for reinitialization if the world changes
        m_urdf_string = urdf_string;
        m_group_name = group_name;
        m_cc_config = config;
        return true;
    }
    else {
        // reset data structures from above
        m_cspace.reset();
        m_grid.reset();
        m_dfield.reset();
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
    ROS_INFO("Registering world observer callback");
    auto ocfn = boost::bind(&CollisionWorldSBPL::worldUpdate, this, _1, _2);
    m_observer_handle = getWorld()->addObserver(ocfn);
}

void CollisionWorldSBPL::worldUpdate(
    const World::ObjectConstPtr& object,
    World::Action action)
{
    ROS_INFO("CollisionWorldSBPL::worldUpdate()");
    if (action & World::ActionBits::UNINITIALIZED) {
        ROS_INFO("  action: UNINITIALIZED");
    }
    else if (action & World::ActionBits::CREATE) {
        ROS_INFO("  action: CREATE");
    }
    else if (action & World::ActionBits::DESTROY) {
        ROS_INFO("  action: DESTROY");
    }
    else if (action & World::ActionBits::MOVE_SHAPE) {
        ROS_INFO("  action: MOVE_SHAPE");
    }
    else if (action & World::ActionBits::ADD_SHAPE) {
        ROS_INFO("  action: ADD_SHAPE");
    }
    else if (action & World::ActionBits::REMOVE_SHAPE)  {
        ROS_INFO("  action: REMOVE_SHAPE");
    }

    ROS_INFO("  id: %s", object->id_.c_str());
    ROS_INFO("  shapes: %zu", object->shapes_.size());
    ROS_INFO("  shape_poses: %zu", object->shape_poses_.size());
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

moveit_msgs::OrientedBoundingBox CollisionWorldSBPL::computeWorldAABB(
    const World& world) const
{
    moveit_msgs::OrientedBoundingBox bb;
    bb.pose.orientation.w = 1.0;
    bb.pose.orientation.x = 0.0;
    bb.pose.orientation.y = 0.0;
    bb.pose.orientation.z = 0.0;

    geometry_msgs::Point min_pt;
    min_pt.x = std::numeric_limits<double>::max();
    min_pt.y = std::numeric_limits<double>::max();
    min_pt.z = std::numeric_limits<double>::max();
    geometry_msgs::Point max_pt;
    max_pt.x = std::numeric_limits<double>::lowest();
    max_pt.y = std::numeric_limits<double>::lowest();
    max_pt.z = std::numeric_limits<double>::lowest();

    if (world.size() == 0) {
        bb.pose.position.x = bb.pose.position.y = bb.pose.position.z = 0.0;
        bb.extents.x = bb.extents.y = bb.extents.z = 0.0;
        return bb;
    }

    for (auto oit = world.begin(); oit != world.end(); ++oit) {
        const World::Object& object = *oit->second;
        const std::string& object_id = object.id_;
        size_t num_shapes = object.shapes_.size();
        ROS_INFO("%zu shapes in object", num_shapes);
        for (size_t i = 0; i < num_shapes; ++i) {
            const Eigen::Affine3d& pose = object.shape_poses_[i];
            shapes::ShapeConstPtr shape = object.shapes_[i];
            Eigen::Vector3d extents = shapes::computeShapeExtents(shape.get());

            const Eigen::Vector3d shape_min =
                    Eigen::Vector3d(pose.translation()) - 0.5 * extents;
            const Eigen::Vector3d shape_max =
                    Eigen::Vector3d(pose.translation()) + 0.5 * extents;

            min_pt.x = std::min(min_pt.x, shape_min.x());
            min_pt.y = std::min(min_pt.y, shape_min.y());
            min_pt.z = std::min(min_pt.z, shape_min.z());
            max_pt.x = std::max(max_pt.x, shape_max.x());
            max_pt.y = std::max(max_pt.y, shape_max.y());
            max_pt.z = std::max(max_pt.z, shape_max.z());
        }
    }

    bb.pose.position.x = 0.5 * (min_pt.x + max_pt.x);
    bb.pose.position.y = 0.5 * (min_pt.y + max_pt.y);
    bb.pose.position.z = 0.5 * (min_pt.z + max_pt.z);
    bb.extents.x = max_pt.x - min_pt.x;
    bb.extents.y = max_pt.y - min_pt.y;
    bb.extents.z = max_pt.z - min_pt.z;
    return bb;
}

bool CollisionWorldSBPL::emptyBoundingBox(
    const moveit_msgs::OrientedBoundingBox& bb) const
{
    return bb.extents.x == 0.0 && bb.extents.y == 0.0 && bb.extents.z == 0.0;
}

} // collision_detection
