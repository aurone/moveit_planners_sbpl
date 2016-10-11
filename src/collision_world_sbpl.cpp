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
#include <eigen_conversions/eigen_msg.h>

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
    ROS_INFO_NAMED(CWP_LOGGER, "CollisionWorldSBPL(const WorldPtr&)");
    construct();
    registerWorldCallback();
}

CollisionWorldSBPL::CollisionWorldSBPL(
    const CollisionWorldSBPL& other,
    const WorldPtr& world)
:
    CollisionWorld(other, world) // copies over the world
{
    ROS_DEBUG_NAMED(CWP_LOGGER, "CollisionWorldSBPL(CollisionWorldSBPL&, const WorldPtr&)");

    m_wcm_config = other.m_wcm_config;
    m_jcgm_map = other.m_jcgm_map;

    m_parent_grid = other.m_grid;
    m_parent_wcm = other.m_wcm;

    m_group_models = other.m_group_models;
    // NOTE: no need to copy observer handle
    registerWorldCallback();
    // NOTE: no need to copy node handle
    m_cspace_pub = other.m_cspace_pub;
}

CollisionWorldSBPL::~CollisionWorldSBPL()
{
    ROS_DEBUG_NAMED(CWP_LOGGER, "~CollisionWorldSBPL()");
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

    clearAllCollisions(res);

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
    clearAllCollisions(res);

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

    clearAllCollisions(res);

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

    clearAllCollisions(res);

    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, robot, state1, state2, acm);
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world) const
{
    ROS_INFO_NAMED(CWP_LOGGER, "checkWorldCollision(req, res, other_world)");

    clearAllCollisions(res);

    // TODO: implement
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world,
    const AllowedCollisionMatrix& acm) const
{
    ROS_INFO_NAMED(CWP_LOGGER, "checkWorldCollision(req, res, other_world, acm)");

    clearAllCollisions(res);

    // TODO: implement
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

    // resolve "world_collision_model" param
    std::string wcm_key;
    if (!ph.searchParam(world_collision_model_param, wcm_key)) {
        const char* msg = "Failed to find 'world_collision_model' key on the param server";
        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
    }

    ROS_INFO_NAMED(CWP_LOGGER, "Found '%s' param at %s", world_collision_model_param, wcm_key.c_str());

    // read parameter
    XmlRpc::XmlRpcValue wcm_config;
    if (!ph.getParam(wcm_key, wcm_config)) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << wcm_key << "' from the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    // check for required type and members
    if (wcm_config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !wcm_config.hasMember("size_x") ||
        !wcm_config.hasMember("size_y") ||
        !wcm_config.hasMember("size_z") ||
        !wcm_config.hasMember("origin_x") ||
        !wcm_config.hasMember("origin_y") ||
        !wcm_config.hasMember("origin_z") ||
        !wcm_config.hasMember("res_m") ||
        !wcm_config.hasMember("max_distance_m"))
    {
        std::stringstream ss;
        ss << "'world_collision_model' param is malformed";
        ROS_ERROR_STREAM(ss.str());
        ROS_ERROR_STREAM("has size_x member " << wcm_config.hasMember("size_x"));
        ROS_ERROR_STREAM("has size_y member " << wcm_config.hasMember("size_y"));
        ROS_ERROR_STREAM("has size_z member " << wcm_config.hasMember("size_z"));
        ROS_ERROR_STREAM("has origin_x member " << wcm_config.hasMember("origin_x"));
        ROS_ERROR_STREAM("has origin_y member " << wcm_config.hasMember("origin_y"));
        ROS_ERROR_STREAM("has origin_z member " << wcm_config.hasMember("origin_z"));
        ROS_ERROR_STREAM("has res_m member " << wcm_config.hasMember("res_m"));
        throw std::runtime_error(ss.str());
    }

    // TODO: more sophisticated parameter checking
    m_wcm_config.size_x = wcm_config["size_x"];
    m_wcm_config.size_y = wcm_config["size_y"];
    m_wcm_config.size_z = wcm_config["size_z"];
    m_wcm_config.origin_x = wcm_config["origin_x"];
    m_wcm_config.origin_y = wcm_config["origin_y"];
    m_wcm_config.origin_z = wcm_config["origin_z"];
    m_wcm_config.res_m = wcm_config["res_m"];
    m_wcm_config.max_distance_m = wcm_config["max_distance_m"];

    LoadJointCollisionGroupMap(ph, m_jcgm_map);

    const bool propagate_negative_distances = false;
    const bool ref_counted = true;
    m_grid = createGridFor(m_wcm_config);

    // TODO: allowed collisions matrix

    m_cspace_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);

    // publish collision world visualizations
    ROS_DEBUG_NAMED(CWP_LOGGER, "Publishing visualization of bounding box");
    auto markers = m_grid->getBoundingBoxVisualization();
    m_cspace_pub.publish(markers);
}

void CollisionWorldSBPL::copyOnWrite()
{
    if (!m_wcm) {
        assert(!m_grid);

        // create our own grid
        const bool propagate_negative_distances = false;
        const bool ref_counted = true;
        m_grid = createGridFor(m_wcm_config);

        // copy over state from parent world collision model
        if (m_parent_wcm) {
            m_wcm = std::make_shared<sbpl::collision::WorldCollisionModel>(
                    *m_parent_wcm, m_grid.get());
        }
    }
}

sbpl::OccupancyGridPtr CollisionWorldSBPL::createGridFor(
    const CollisionWorldConfig& config) const
{
    ROS_DEBUG_NAMED(CWP_LOGGER, "  Creating Distance Field");
    ROS_DEBUG_NAMED(CWP_LOGGER, "    size: (%0.3f, %0.3f, %0.3f)", config.size_x, config.size_y, config.size_z);
    ROS_DEBUG_NAMED(CWP_LOGGER, "    origin: (%0.3f, %0.3f, %0.3f)", config.origin_x, config.origin_y, config.origin_z);
    ROS_DEBUG_NAMED(CWP_LOGGER, "    resolution: %0.3f", config.res_m);
    ROS_DEBUG_NAMED(CWP_LOGGER, "    max_distance: %0.3f", config.max_distance_m);

    const bool propagate_negative_distances = false;
    const bool ref_counted = true;
    return std::make_shared<sbpl::OccupancyGrid>(
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
}

std::string CollisionWorldSBPL::groupModelName(
    const std::string& robot_name,
    const std::string& group_name) const
{
    return robot_name + "." + group_name;
}

std::vector<double> CollisionWorldSBPL::getCheckedVariables(
    const GroupModel& gm,
    const moveit::core::RobotState& state) const
{
    if (gm.are_variables_contiguous) {
        return std::vector<double>(
                state.getVariablePositions() + gm.variables_offset,
                state.getVariablePositions() + gm.variables_offset + gm.variable_names.size());
    }
    else {
        std::vector<double> vars;
        for (size_t i = 0; i < gm.variable_indices.size(); ++i) {
            vars.push_back(state.getVariablePosition(gm.variable_indices[i]));
        }
        return vars;
    }
}

CollisionWorldSBPL::GroupModelPtr CollisionWorldSBPL::getGroupModel(
    const CollisionRobotSBPL& collision_robot,
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name)
{
    // return an existing group model if we've instantiated one for this group
    std::string group_model_name = groupModelName(robot_model.getName(), group_name);
    auto it = m_group_models.find(group_model_name);
    if (it != m_group_models.end()) {
        return it->second;
    }

    ROS_INFO_NAMED(CWP_LOGGER, "Creating Group Model '%s'", group_model_name.c_str());

    GroupModelPtr group_model = std::make_shared<GroupModel>();

    /////////////////
    // Robot Model //
    /////////////////

    ROS_DEBUG_NAMED(CWP_LOGGER, "  Initializing Robot Model");
    ExtractRoboVariables(
            robot_model,
            group_model->variable_names,
            group_model->variable_indices,
            group_model->are_variables_contiguous,
            group_model->variables_offset);

    const auto rcm = collision_robot.robotCollisionModel();

    ////////////////////
    // Distance Field //
    ////////////////////

    double df_max_distance_m = getSelfCollisionPropagationDistance(*rcm);

    // TODO: this can be substantially smaller since it only has to encompass
    // the range of motion of the robot

    group_model->grid = createGridFor(m_wcm_config);

    auto grid = group_model->grid;

    grid->setReferenceFrame(robot_model.getModelFrame());

    /////////////////////
    // Collision Space //
    /////////////////////

    ROS_DEBUG_NAMED(CWP_LOGGER, "  Constructing Collision Space");

    auto jgcgit = m_jcgm_map.find(group_name);
    std::string collision_group_name;
    if (jgcgit == m_jcgm_map.end()) {
        collision_group_name = group_name;
    }
    else {
        collision_group_name = jgcgit->second;
    }

    ROS_DEBUG_NAMED(CWP_LOGGER, "Constructing cspace for group '%s'", collision_group_name.c_str());

    sbpl::collision::CollisionSpaceBuilder builder;
    auto cspace = builder.build(
            grid.get(),
            rcm,
            collision_group_name,
            group_model->variable_names);

    if (!cspace) {
        ROS_ERROR_NAMED(CWP_LOGGER, "  Failed to initialize collision space");
        return GroupModelPtr();
    }

    group_model->cspace = cspace;

    ROS_DEBUG_NAMED(CWP_LOGGER, "  Successfully built collision space");

    // store the successfully initialized group model
    m_group_models[group_model_name] = group_model;
    return group_model;
}

const distance_field::PropagationDistanceField*
CollisionWorldSBPL::distanceField(
    const std::string& robot_name,
    const std::string& group_name) const
{
    const std::string& group_model_name = groupModelName(robot_name, group_name);
    auto it = m_group_models.find(group_model_name);
    if (it == m_group_models.end()) {
        return nullptr;
    }
    else {
        return it->second->grid->getDistanceField().get();
    }
}

void CollisionWorldSBPL::registerWorldCallback()
{
    ROS_DEBUG_NAMED(CWP_LOGGER, "Registering world observer callback");
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

void CollisionWorldSBPL::addWorldToCollisionSpace(
    const World& world,
    sbpl::collision::CollisionSpace& cspace)
{
    for (auto oit = world.begin(); oit != world.end(); ++oit) {
        ROS_DEBUG_NAMED(CWP_LOGGER, "Adding object '%s' to the configuration space", oit->first.c_str());
        assert(oit->second.get());
        cspace.insertObject(oit->second);
    }
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state)
{
    // TODO: implement
    ROS_ERROR_NAMED(CWP_LOGGER, "checkRobotCollision(req, res, robot, state)");
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    const CollisionRobotSBPL& crobot = (const CollisionRobotSBPL&)robot;
    const RobotCollisionModelConstPtr& rcm = crobot.robotCollisionModel();
    if (state.getRobotModel()->getName() != rcm->name()) {
        ROS_ERROR_NAMED(CWP_LOGGER, "Collision Robot Model does not match Robot Model");
        setVacuousCollision(res);
        return;
    }

    auto gm = getGroupModel(crobot, *state.getRobotModel(), req.group_name);
    if (!gm) {
        ROS_ERROR_NAMED(CWP_LOGGER, "Failed to get Group Model for robot '%s', group '%s'", state.getRobotModel()->getName().c_str(), req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    const Eigen::Affine3d& T_model_robot =
            state.getGlobalLinkTransform(state.getRobotModel()->getRootLink());
    cspace->setWorldToModelTransform(T_model_robot);

    std::vector<double> vars = getCheckedVariables(*gm, state);

    double dist;
    const bool verbose = req.verbose;
    const bool visualize = req.verbose;
    bool valid = cspace->checkCollision(
            vars, AllowedCollisionMatrixInterface(acm), verbose, visualize, dist);
    if (visualize) {
        auto markers = cspace->getCollisionRobotVisualization(vars);
        if (!valid) {
            for (auto& m : markers.markers) {
                m.color.r = 1.0;
                m.color.g = m.color.b = 0.0;
            }
        }
        m_cspace_pub.publish(markers);
    }

    res.collision = !valid;
    if (req.distance) {
        res.distance = dist;
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
}

double CollisionWorldSBPL::getSelfCollisionPropagationDistance(
    const sbpl::collision::RobotCollisionModel& rcm) const
{
    return 0.0;
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

bool CollisionWorldSBPL::worldObjectToCollisionObjectMsgFull(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object) const
{
    moveit_msgs::CollisionObject obj_msg;

    obj_msg.header.stamp = ros::Time(0);

    // TODO: safe to assume that the world frame will always be the world
    // frame or should we try to transform things here somehow
//    obj_msg.header.frame_id = m_wcm_config.world_frame;

    obj_msg.id = object.id_;

    assert(object.shape_poses_.size() == object.shapes_.size());
    for (size_t sind = 0; sind < object.shapes_.size(); ++sind) {
        const Eigen::Affine3d& shape_transform = object.shape_poses_[sind];
        const shapes::ShapeConstPtr& shape = object.shapes_[sind];

        // convert shape to corresponding shape_msgs type
        switch (shape->type) {
        case shapes::UNKNOWN_SHAPE:
        {
            ROS_WARN_NAMED(CWP_LOGGER, "Object '%s' contains shape of unknown type", object.id_.c_str());
            return false;
        }   break;
        case shapes::SPHERE:
        {
            const shapes::Sphere* sphere =
                    dynamic_cast<const shapes::Sphere*>(shape.get());

            shape_msgs::SolidPrimitive prim;
            prim.type = shape_msgs::SolidPrimitive::SPHERE;
            prim.dimensions.resize(1);
            prim.dimensions[0] = sphere->radius;
            obj_msg.primitives.push_back(prim);

            geometry_msgs::Pose pose;
            tf::poseEigenToMsg(shape_transform, pose);
            obj_msg.primitive_poses.push_back(pose);
        }   break;
        case shapes::CYLINDER:
        {
            const shapes::Cylinder* cylinder =
                    dynamic_cast<const shapes::Cylinder*>(shape.get());

            shape_msgs::SolidPrimitive prim;
            prim.type = shape_msgs::SolidPrimitive::CYLINDER;
            prim.dimensions.resize(2);
            prim.dimensions[0] = cylinder->radius;
            prim.dimensions[1] = cylinder->length;
            obj_msg.primitives.push_back(prim);

            geometry_msgs::Pose pose;
            tf::poseEigenToMsg(shape_transform, pose);
            obj_msg.primitive_poses.push_back(pose);
        }   break;
        case shapes::CONE:
        {
            ROS_ERROR_NAMED(CWP_LOGGER, "Unsupported object type: Cone");
        }   break;
        case shapes::BOX:
        {
            const shapes::Box* box =
                    dynamic_cast<const shapes::Box*>(shape.get());

            shape_msgs::SolidPrimitive prim;
            prim.type = shape_msgs::SolidPrimitive::BOX;
            prim.dimensions.resize(3);
            prim.dimensions[0] = box->size[0];
            prim.dimensions[1] = box->size[1];
            prim.dimensions[2] = box->size[2];
            obj_msg.primitives.push_back(prim);

            geometry_msgs::Pose pose;
            tf::poseEigenToMsg(shape_transform, pose);
            obj_msg.primitive_poses.push_back(pose);
        }   break;
        case shapes::PLANE:
        {
            ROS_ERROR_NAMED(CWP_LOGGER, "Unsupported object type: Plane");
        }   break;
        case shapes::MESH:
        {
            const shapes::Mesh* mesh =
                    dynamic_cast<const shapes::Mesh*>(shape.get());

            obj_msg.meshes.push_back(shape_msgs::Mesh());
            shape_msgs::Mesh& mesh_msg = obj_msg.meshes.back();

            // convert shapes::Mesh to shape_msgs::Mesh
            mesh_msg.vertices.resize(mesh->vertex_count);
            for (int i = 0; i < mesh->vertex_count; ++i) {
                mesh_msg.vertices[i].x = mesh->vertices[3 * i + 0];
                mesh_msg.vertices[i].y = mesh->vertices[3 * i + 1];
                mesh_msg.vertices[i].z = mesh->vertices[3 * i + 2];
            }

            mesh_msg.triangles.resize(mesh->triangle_count);
            for (int i = 0; i < mesh->triangle_count; ++i) {
                mesh_msg.triangles[i].vertex_indices[0] = mesh->triangles[3 * i + 0];
                mesh_msg.triangles[i].vertex_indices[1] = mesh->triangles[3 * i + 1];
                mesh_msg.triangles[i].vertex_indices[2] = mesh->triangles[3 * i + 2];
            }

            geometry_msgs::Pose pose;
            tf::poseEigenToMsg(shape_transform, pose);
            obj_msg.mesh_poses.push_back(pose);
        }   break;
        case shapes::OCTREE:
        {
            ROS_ERROR_NAMED(CWP_LOGGER, "Unsupported object type: OcTree");
        }   break;
        }
    }

    collision_object = std::move(obj_msg);
    return true;
}

bool CollisionWorldSBPL::worldObjectToCollisionObjectMsgName(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object) const
{
    moveit_msgs::CollisionObject obj_msg;
    obj_msg.header.stamp = ros::Time(0);
//    obj_msg.header.frame_id = m_wcm_config.world_frame;
    obj_msg.id = object.id_;
    collision_object = std::move(obj_msg);
    return true;
}

} // collision_detection
