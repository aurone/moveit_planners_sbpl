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

#include <moveit_planners_sbpl/collision_world_sbpl.h>

// standard includes
#include <stdexcept>

// system includes
#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>
#include <leatherman/print.h>

// project includes
#include <moveit_planners_sbpl/moveit_robot_model.h>

// module includes
#include <moveit_planners_sbpl/collision_common_sbpl.h>

namespace collision_detection {

// cwp = collision world plugin
static const char* CWP_LOGGER = "world_collisions";

CollisionWorldSBPL::CollisionWorldSBPL() : CollisionWorld()
{
    ROS_INFO_NAMED(CWP_LOGGER, "CollisionWorldSBPL()");
    construct();
}

CollisionWorldSBPL::CollisionWorldSBPL(const WorldPtr& world) :
    CollisionWorld(world)
{
    ROS_INFO_NAMED(CWP_LOGGER, "CollisionWorldSBPL(world = %p)", world.get());
    construct();
    registerWorldCallback();
}

CollisionWorldSBPL::CollisionWorldSBPL(
    const CollisionWorldSBPL& other,
    const WorldPtr& world)
:
    CollisionWorld(other, world) // copies over the world
{
//    ROS_DEBUG_NAMED(CWP_LOGGER, "CollisionWorldSBPL(other = %p, world = %p)", &other, world.get());

    m_wcm_config = other.m_wcm_config;
    m_jcgm_map = other.m_jcgm_map;

    m_parent_grid = other.m_grid ? other.m_grid : other.m_parent_grid;
    m_parent_wcm = other.m_wcm ? other.m_wcm : other.m_parent_wcm;

    m_updaters = other.m_updaters;
    // NOTE: no need to copy observer handle
    registerWorldCallback();
    // NOTE: no need to copy node handle
    m_cspace_pub = other.m_cspace_pub;
}

CollisionWorldSBPL::~CollisionWorldSBPL()
{
//    ROS_DEBUG_NAMED(CWP_LOGGER, "~CollisionWorldSBPL()");
    const WorldPtr& curr_world = getWorld();
    if (curr_world) {
        curr_world->removeObserver(m_observer_handle);
    }
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state) const
{
    ROS_INFO_NAMED(CWP_LOGGER, "checkRobotCollision(req, res, robot, state)");

    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, robot, state);
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, robot, state, acm);
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    ROS_INFO_NAMED(CWP_LOGGER, "checkRobotCollision(req, res, robot, state1, state2)");

    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, robot, state1, state2);
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    ROS_INFO_NAMED(CWP_LOGGER, "checkRobotCollision(req, res, robot, state1, state2, acm)");

    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, robot, state1, state2, acm);
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world) const
{
    ROS_INFO_NAMED(CWP_LOGGER, "checkWorldCollision(req, res, other_world)");
    // TODO: implement
    setVacuousCollision(res);
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world,
    const AllowedCollisionMatrix& acm) const
{
    ROS_INFO_NAMED(CWP_LOGGER, "checkWorldCollision(req, res, other_world, acm)");
    // TODO: implement
    setVacuousCollision(res);
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    ROS_INFO_NAMED(CWP_LOGGER, "distanceRobot(robot, state)");
    return -1.0;
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    ROS_INFO_NAMED(CWP_LOGGER, "distanceRobot(robot, state, acm)");
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(const CollisionWorld& world) const
{
    // TODO: implement
    ROS_INFO_NAMED(CWP_LOGGER, "distanceWorld(world)");
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(
    const CollisionWorld& world,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    ROS_INFO_NAMED(CWP_LOGGER, "distanceWorld(world, acm)");
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
    ROS_INFO_NAMED(CWP_LOGGER, "setWorld(const WorldPtr&)");

    registerWorldCallback();
}

void CollisionWorldSBPL::construct()
{
    ros::NodeHandle ph("~");

    const char* world_collision_model_param = "world_collision_model";
    LoadCollisionGridConfig(ph, world_collision_model_param, m_wcm_config);

    LoadJointCollisionGroupMap(ph, m_jcgm_map);

    m_grid = createGridFor(m_wcm_config);
    m_wcm = std::make_shared<sbpl::collision::WorldCollisionModel>(m_grid.get());

    // TODO: allowed collisions matrix

    ros::NodeHandle nh;
    m_cspace_pub = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 10);

    ROS_INFO("Sleep to allow publish to set up");
    ros::Duration(0.5).sleep();
    ROS_INFO("Done sleeping");

    // publish collision world visualizations
    auto markers = m_grid->getBoundingBoxVisualization();
    m_cspace_pub.publish(markers);
}

void CollisionWorldSBPL::copyOnWrite()
{
    if (!m_wcm) {
        ROS_DEBUG_NAMED(CWP_LOGGER, "Spawn derivative world collision model");
        assert(!m_grid);

        // create our own grid
        m_grid = createGridFor(m_wcm_config);

        // copy over state from parent world collision model
        if (m_parent_wcm) {
            m_wcm = std::make_shared<sbpl::collision::WorldCollisionModel>(
                    *m_parent_wcm, m_grid.get());

            m_parent_grid.reset();
            m_parent_wcm.reset();
        }
    }
}

sbpl::OccupancyGridPtr CollisionWorldSBPL::createGridFor(
    const CollisionGridConfig& config) const
{
    ROS_DEBUG_NAMED(CWP_LOGGER, "  Creating Distance Field");
    ROS_DEBUG_NAMED(CWP_LOGGER, "    size: (%0.3f, %0.3f, %0.3f)", config.size_x, config.size_y, config.size_z);
    ROS_DEBUG_NAMED(CWP_LOGGER, "    origin: (%0.3f, %0.3f, %0.3f)", config.origin_x, config.origin_y, config.origin_z);
    ROS_DEBUG_NAMED(CWP_LOGGER, "    resolution: %0.3f", config.res_m);
    ROS_DEBUG_NAMED(CWP_LOGGER, "    max_distance: %0.3f", config.max_distance_m);

    const bool propagate_negative_distances = false;
    const bool ref_counted = true;
    auto dmap = std::make_shared<sbpl::OccupancyGrid>(
            config.size_x,
            config.size_y,
            config.size_z,
            config.res_m,
            config.origin_x,
            config.origin_y,
            config.origin_z,
            config.max_distance_m,
            propagate_negative_distances,
            ref_counted);
    dmap->setReferenceFrame(config.frame_id);
    return dmap;
}

CollisionStateUpdaterPtr CollisionWorldSBPL::getCollisionStateUpdater(
    const CollisionRobotSBPL& collision_robot,
    const moveit::core::RobotModel& robot_model)
{
    // return an existing updater if available
    auto it = m_updaters.find(robot_model.getName());
    if (it != m_updaters.end()) {
        return it->second;
    }

    ROS_INFO_NAMED(CWP_LOGGER, "Create Collision State Updater for '%s'", robot_model.getName().c_str());

    auto gm = std::make_shared<CollisionStateUpdater>();
    if (!gm->init(robot_model, collision_robot.robotCollisionModel())) {
        return CollisionStateUpdaterPtr();
    }

    // store the successfully initialized group model
    m_updaters[robot_model.getName()] = gm;
    return gm;
}

const distance_field::DistanceField* CollisionWorldSBPL::distanceField(
    const std::string& robot_name,
    const std::string& group_name) const
{
    if (m_grid) {
        return m_grid->getDistanceField().get();
    } else if (m_parent_grid) {
        return m_parent_grid->getDistanceField().get();
    } else {
        return nullptr;
    }
}

void CollisionWorldSBPL::registerWorldCallback()
{
//    ROS_DEBUG_NAMED(CWP_LOGGER, "Registering world observer callback");
    auto ocfn = boost::bind(&CollisionWorldSBPL::worldUpdate, this, _1, _2);
    m_observer_handle = getWorld()->addObserver(ocfn);
}

void CollisionWorldSBPL::worldUpdate(
    const World::ObjectConstPtr& object,
    World::Action action)
{
    ROS_INFO_NAMED(CWP_LOGGER, "CollisionWorldSBPL::worldUpdate()");
    ROS_INFO_NAMED(CWP_LOGGER, "  id: %s", object->id_.c_str());
    ROS_INFO_NAMED(CWP_LOGGER, "  shapes: %zu", object->shapes_.size());
    ROS_INFO_NAMED(CWP_LOGGER, "  shape_poses: %zu", object->shape_poses_.size());
    if (action & World::ActionBits::UNINITIALIZED) {
        ROS_INFO_NAMED(CWP_LOGGER, "  action: UNINITIALIZED");
        processWorldUpdateUninitialized(object);
    }
    else if (action & World::ActionBits::CREATE) {
        ROS_INFO_NAMED(CWP_LOGGER, "  action: CREATE");
        processWorldUpdateCreate(object);
    }
    else if (action & World::ActionBits::DESTROY) {
        ROS_INFO_NAMED(CWP_LOGGER, "  action: DESTROY");
        processWorldUpdateDestroy(object);
    }
    else if (action & World::ActionBits::MOVE_SHAPE) {
        ROS_INFO_NAMED(CWP_LOGGER, "  action: MOVE_SHAPE");
        processWorldUpdateMoveShape(object);
    }
    else if (action & World::ActionBits::ADD_SHAPE) {
        ROS_INFO_NAMED(CWP_LOGGER, "  action: ADD_SHAPE");
        processWorldUpdateAddShape(object);
    }
    else if (action & World::ActionBits::REMOVE_SHAPE)  {
        ROS_INFO_NAMED(CWP_LOGGER, "  action: REMOVE_SHAPE");
        processWorldUpdateRemoveShape(object);
    }
}

void CollisionWorldSBPL::setVacuousCollision(CollisionResult& res) const
{
    res.collision = true;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 0.0;
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
        ROS_DEBUG_NAMED(CWP_LOGGER, "%zu shapes in object", num_shapes);
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

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state)
{
    // TODO: implement
    ROS_ERROR_NAMED(CWP_LOGGER, "checkRobotCollision(req, res, robot, state)");
    setVacuousCollision(res);
}

/// Note: The output CollisionResult is shared between multiple collision
/// checking calls (i.e. both for world and self collisions). The policy is to
/// only set fields when a collision occurs and not to clear fields.
void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    const CollisionRobotSBPL& crobot = (const CollisionRobotSBPL&)robot;
    const auto& rcm = crobot.robotCollisionModel();
    if (state.getRobotModel()->getName() != rcm->name()) {
        ROS_ERROR_NAMED(CWP_LOGGER, "Collision Robot Model does not match Robot Model");
        setVacuousCollision(res);
        return;
    }

    auto gm = getCollisionStateUpdater(crobot, *state.getRobotModel());
    if (!gm) {
        ROS_ERROR_NAMED(CWP_LOGGER, "Failed to get Group Model for robot '%s', group '%s'", state.getRobotModel()->getName().c_str(), req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto jgcgit = m_jcgm_map.find(req.group_name);
    const std::string& collision_group_name =
            jgcgit == m_jcgm_map.end() ? req.group_name : jgcgit->second;

    if (!rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(CWP_LOGGER, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    int gidx = rcm->groupIndex(collision_group_name);

    gm->update(state);

    sbpl::collision::WorldCollisionModelConstPtr ewcm;
    if (m_wcm) {
        ewcm = m_wcm;
    } else if (m_parent_wcm) {
        ewcm = m_parent_wcm;
    } else {
        ROS_ERROR_NAMED(CWP_LOGGER, "Neither local nor parent world collision model valid");
        setVacuousCollision(res);
        return;
    }

    double dist;
    bool valid = ewcm->checkCollision(
            *gm->collisionState(),
            *gm->attachedBodiesCollisionState(),
            gidx,
            dist);

    ROS_INFO_STREAM_COND_NAMED(req.verbose, CWP_LOGGER, "valid: " << std::boolalpha << valid << ", dist: " << dist);
    ROS_DEBUG_STREAM_COND_NAMED(!req.verbose, CWP_LOGGER, "valid: " << std::boolalpha << valid << ", dist: " << dist);

    const bool visualize = req.verbose;
    if (visualize) {
        auto ma = getCollisionRobotVisualization(
                *gm->collisionState(),
                *gm->attachedBodiesCollisionState(),
                gidx);
        if (!valid) {
            for (auto& m : ma.markers) {
                m.color.r = 1.0;
                m.color.g = m.color.b = 0.0;
            }
        }
        m_cspace_pub.publish(ma);
    }

    if (!valid) {
        res.collision = true;
    }
    if (req.distance) {
        res.distance = std::min(res.distance, dist);
    }
    if (req.cost) {
        ROS_WARN_ONCE("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_WARN_ONCE("Contacts not computed by sbpl collision checker");
    }
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2)
{
    // TODO: implement
    ROS_ERROR_NAMED(CWP_LOGGER, "checkRobotCollision(req, res, robot, state1, state2)");
    setVacuousCollision(res);
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm)
{
    // TODO: implement
    ROS_ERROR_NAMED(CWP_LOGGER, "checkRobotCollision(req, res, robot, state1, state2, acm)");
    setVacuousCollision(res);
}

void CollisionWorldSBPL::processWorldUpdateUninitialized(
    const World::ObjectConstPtr& object)
{

}

void CollisionWorldSBPL::processWorldUpdateCreate(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    m_wcm->insertObject(object);
}

void CollisionWorldSBPL::processWorldUpdateDestroy(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    m_wcm->removeObject(object);
}

void CollisionWorldSBPL::processWorldUpdateMoveShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    m_wcm->moveShapes(object);
}

void CollisionWorldSBPL::processWorldUpdateAddShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    m_wcm->insertShapes(object);
}

void CollisionWorldSBPL::processWorldUpdateRemoveShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    m_wcm->removeShapes(object);
}

visualization_msgs::MarkerArray
CollisionWorldSBPL::getCollisionRobotVisualization(
    sbpl::collision::RobotCollisionState& rcs,
    sbpl::collision::AttachedBodiesCollisionState& abcs,
    int gidx) const
{
    auto ma = GetCollisionMarkers(rcs, abcs, gidx);
    for (auto& m : ma.markers) {
        m.ns = "world_collision";
        if (m_grid) {
            m.header.frame_id = m_grid->getReferenceFrame();
        } else if (m_parent_grid) {
            m.header.frame_id = m_parent_grid->getReferenceFrame();
        }
    }
    return ma;
}

} // collision_detection
