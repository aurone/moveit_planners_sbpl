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

namespace collision_detection {

// cdp = collision detection plugin
static const char* CDP_LOGGER = "collisions";

CollisionWorldSBPL::CollisionWorldSBPL() : CollisionWorld()
{
    ROS_INFO("CollisionWorldSBPL()");
    construct();
}

CollisionWorldSBPL::CollisionWorldSBPL(const WorldPtr& world) :
    CollisionWorld(world)
{
    ROS_INFO("CollisionWorldSBPL(const WorldPtr&)");
    construct();
    registerWorldCallback();
}

CollisionWorldSBPL::CollisionWorldSBPL(
    const CollisionWorldSBPL& other,
    const WorldPtr& world)
:
    CollisionWorld(other, world) // copies over the world
{
    ROS_DEBUG("CollisionWorldSBPL(CollisionWorldSBPL&, const WorldPtr&)");

    m_world_collision_model_config = other.m_world_collision_model_config;
    m_group_models = other.m_group_models;
    // NOTE: no need to copy observer handle
    registerWorldCallback();
    // NOTE: no need to copy node handle
    m_cspace_pub = other.m_cspace_pub;
}

CollisionWorldSBPL::~CollisionWorldSBPL()
{
    ROS_DEBUG("~CollisionWorldSBPL()");
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
    ROS_INFO("checkRobotCollision(req, res, robot, state)");

    if (checkDegenerateCollision(res)) {
        return;
    }

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
    if (checkDegenerateCollision(res)) {
        return;
    }

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
    ROS_INFO("checkRobotCollision(req, res, robot, state1, state2)");

    if (checkDegenerateCollision(res)) {
        return;
    }

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
    ROS_INFO("checkRobotCollision(req, res, robot, state1, state2, acm)");

    if (checkDegenerateCollision(res)) {
        return;
    }

    clearAllCollisions(res);

    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, robot, state1, state2, acm);
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world) const
{
    ROS_INFO("checkWorldCollision(req, res, other_world)");

    if (checkDegenerateCollision(res)) {
        return;
    }

    clearAllCollisions(res);

    // TODO: implement
}

void CollisionWorldSBPL::checkWorldCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionWorld& other_world,
    const AllowedCollisionMatrix& acm) const
{
    ROS_INFO("checkWorldCollision(req, res, other_world, acm)");

    if (checkDegenerateCollision(res)) {
        return;
    }

    clearAllCollisions(res);

    // TODO: implement
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    ROS_INFO("distanceRobot(robot, state)");
    return -1.0;
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    ROS_INFO("distanceRobot(robot, state, acm)");
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(const CollisionWorld& world) const
{
    // TODO: implement
    ROS_INFO("distanceWorld(world)");
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(
    const CollisionWorld& world,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    ROS_INFO("distanceWorld(world, acm)");
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

void CollisionWorldSBPL::construct()
{
    ros::NodeHandle ph("~");

    const char* world_collision_model_param = "world_collision_model";

    std::string wcm_key;
    if (!ph.searchParam(world_collision_model_param, wcm_key)) {
        const char* msg = "Failed to find 'world_collision_model' key on the param server";
        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
    }

    ROS_INFO_NAMED(CDP_LOGGER, "Found '%s' param at %s", world_collision_model_param, wcm_key.c_str());

    // load occupancy grid parameters
    XmlRpc::XmlRpcValue wcm_config;
    if (!ph.getParam(wcm_key, wcm_config)) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << wcm_key << "' from the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

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
        ROS_ERROR_STREAM("has max_distance_m member " << wcm_config.hasMember("max_distance_m"));
        throw std::runtime_error(ss.str());
    }

    // TODO: more sophisticated parameter checking
    m_world_collision_model_config.world_frame = "world"; // TODO: scene.getPlanningFrame();
    m_world_collision_model_config.size_x = wcm_config["size_x"];
    m_world_collision_model_config.size_y = wcm_config["size_y"];
    m_world_collision_model_config.size_z = wcm_config["size_z"];
    m_world_collision_model_config.origin_x = wcm_config["origin_x"];
    m_world_collision_model_config.origin_y = wcm_config["origin_y"];
    m_world_collision_model_config.origin_z = wcm_config["origin_z"];
    m_world_collision_model_config.res_m = wcm_config["res_m"];
    m_world_collision_model_config.max_distance_m = wcm_config["max_distance_m"];

    m_cspace_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);

    // TODO: allowed collisions matrix
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

bool CollisionWorldSBPL::getRobotVariableNames(
    const moveit::core::RobotModel& model,
    std::vector<std::string>& var_names,
    std::vector<int>& var_indices) const
{
    // traverse the robot kinematic structure to gather all the joint variables
    const moveit::core::LinkModel* root_link = model.getRootLink();
    if (!root_link) {
        return false;
    }

    // get all descendant joint models
    std::vector<const moveit::core::JointModel*> robot_joint_models;
    for (auto joint_model : root_link->getChildJointModels()) {
        robot_joint_models.insert(
                robot_joint_models.end(),
                joint_model->getDescendantJointModels().begin(),
                joint_model->getDescendantJointModels().end());
    }
    robot_joint_models.insert(
            robot_joint_models.end(),
            root_link->getChildJointModels().begin(),
            root_link->getChildJointModels().end());

    // aggregate all variable names
    std::vector<std::string> variable_names;
    for (auto joint_model : robot_joint_models) {
        variable_names.insert(
                variable_names.end(),
                joint_model->getVariableNames().begin(),
                joint_model->getVariableNames().end());
    }

    // sort by their order in the robot model variable vector
    std::sort(variable_names.begin(), variable_names.end(),
            [&](const std::string& var_a, const std::string& var_b)
            {
                size_t va_idx = std::distance(
                        model.getVariableNames().begin(),
                        std::find(model.getVariableNames().begin(), model.getVariableNames().end(), var_a));
                size_t vb_idx = std::distance(
                        model.getVariableNames().begin(),
                        std::find(model.getVariableNames().begin(), model.getVariableNames().end(), var_b));
                return va_idx < vb_idx;
            });


    std::vector<int> variable_indices;
    for (const auto& var_name : variable_names) {
        variable_indices.push_back(std::distance(
                model.getVariableNames().begin(),
                std::find(model.getVariableNames().begin(), model.getVariableNames().end(), var_name)));
    }

    var_names = std::move(variable_names);
    var_indices = std::move(variable_indices);
    return true;
}

CollisionWorldSBPL::GroupModelPtr CollisionWorldSBPL::getGroupModel(
    const CollisionRobotSBPL& collision_robot,
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name)
{
    std::string group_model_name = groupModelName(robot_model.getName(), group_name);
    auto it = m_group_models.find(group_model_name);
    if (it != m_group_models.end()) {
        return it->second;
    }

    ROS_INFO("Creating Group Model '%s'", group_model_name.c_str());

    GroupModelPtr group_model = std::make_shared<GroupModel>();

    /////////////////
    // Robot Model //
    /////////////////

    ROS_DEBUG_NAMED(CDP_LOGGER, "  Initializing Robot Model");
    initializeRobotModel(*group_model, robot_model);

    ////////////////////
    // Distance Field //
    ////////////////////

    // TODO: should be dependent on size of the largest sphere in the collision
    // group model (don't forget about attached objects, way to set this
    // explicitly on the collision space or the occupancy grid?)
    const double df_max_distance_m = 0.2;

    const double df_size_x = m_world_collision_model_config.size_x;
    const double df_size_y = m_world_collision_model_config.size_y;
    const double df_size_z = m_world_collision_model_config.size_z;
    const double df_res_m = m_world_collision_model_config.res_m;
    const double df_origin_x = m_world_collision_model_config.origin_x;
    const double df_origin_y = m_world_collision_model_config.origin_y;
    const double df_origin_z = m_world_collision_model_config.origin_z;

    ROS_DEBUG_NAMED(CDP_LOGGER, "  Creating Distance Field");
    ROS_DEBUG_NAMED(CDP_LOGGER, "    size: (%0.3f, %0.3f, %0.3f)", df_size_x, df_size_y, df_size_z);
    ROS_DEBUG_NAMED(CDP_LOGGER, "    origin: (%0.3f, %0.3f, %0.3f)", df_origin_x, df_origin_y, df_origin_z);
    ROS_DEBUG_NAMED(CDP_LOGGER, "    resolution: %0.3f", df_res_m);
    ROS_DEBUG_NAMED(CDP_LOGGER, "    max_distance: %0.3f", df_max_distance_m);

    const bool propagate_negative_distances = false;
    const bool ref_counted = true;
    group_model->grid = std::make_shared<sbpl::OccupancyGrid>(
            df_size_x, df_size_y, df_size_z,
            df_res_m,
            df_origin_x, df_origin_y, df_origin_z,
            df_max_distance_m,
            propagate_negative_distances,
            ref_counted);

    auto grid = group_model->grid;

    grid->setReferenceFrame(m_world_collision_model_config.world_frame);

    /////////////////////
    // Collision Space //
    /////////////////////

    ROS_DEBUG_NAMED(CDP_LOGGER, "  Constructing Collision Space");

    const auto rcm = collision_robot.robotCollisionModel();

    sbpl::collision::CollisionSpaceBuilder builder;
    group_model->cspace = builder.build(
            grid.get(), rcm, group_name, group_model->variable_names);
    auto cspace = group_model->cspace;

    if (!cspace) {
        ROS_ERROR("  Failed to initialize collision space");
        return GroupModelPtr();
    }

    ROS_DEBUG_NAMED(CDP_LOGGER, "  Successfully built collision space");

    ROS_DEBUG_NAMED(CDP_LOGGER, "  Adding world to collision space");
    WorldPtr curr_world = getWorld();
    if (curr_world) {
        // TODO: originally the intention was to automatically determine the
        // distance field boundaries inherited from the world bounding box.
        // Currently this doesn't seem like the thing to do anymore, since the
        // distance field extents are also considered boundaries for collision
        // checking. Warrants review, since the workspace boundaries in the
        // planning request seem like the proper way to express boundaries for
        // the robot
        ROS_DEBUG_NAMED(CDP_LOGGER, "  Computing world bounding box");
        moveit_msgs::OrientedBoundingBox world_bb = computeWorldAABB(*curr_world);
        ROS_DEBUG_NAMED(CDP_LOGGER, "  -> Bounding Box:");
        ROS_DEBUG_NAMED(CDP_LOGGER, "     position: (%0.3f, %0.3f, %0.3f)", world_bb.pose.position.x, world_bb.pose.position.y, world_bb.pose.position.z);
        ROS_DEBUG_NAMED(CDP_LOGGER, "     extents: (%0.3f, %0.3f, %0.3f)", world_bb.extents.x, world_bb.extents.y, world_bb.extents.z);

        addWorldToCollisionSpace(*curr_world, *cspace);
    }
    ROS_DEBUG_NAMED(CDP_LOGGER, "  Added world to collision space");

    // publish collision world visualizations
    ROS_DEBUG_NAMED(CDP_LOGGER, "Publishing visualization of bounding box");
    visualization_msgs::MarkerArray markers;
    markers = cspace->getBoundingBoxVisualization();
    m_cspace_pub.publish(markers);

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

void CollisionWorldSBPL::initializeRobotModel(
    GroupModel& group_model,
    const moveit::core::RobotModel& robot_model)
{
    // Figure out how to extract the internal robot state from a complete robot
    // state. We're just going to send the entire internal robot state variables
    // to isStateValid calls since there isn't a notion of planning variables
    // here. The other robot state variables will be set via the world to model
    // transform in the collision space

    // get all variables that are descendants of the root link
    std::vector<std::string> robot_var_names;
    std::vector<int> robot_var_indices;
    if (!getRobotVariableNames(robot_model, robot_var_names, robot_var_indices))
    {
        // shouldn't happen
        throw std::runtime_error("Failed to extract robot variable names!");
    }

    assert(robot_var_names.size() == robot_var_indices.size());
    // could also assert that the robot variable indices are sorted here

    bool contiguous = true;
    for (size_t i = 1; i < robot_var_indices.size(); ++i) {
        if (robot_var_indices[i] != robot_var_indices[i - 1] + 1) {
            contiguous = false;
            break;
        }
    }

    group_model.variable_names = robot_var_names;
    group_model.variable_indices = robot_var_indices;
    group_model.are_variables_contiguous = contiguous;
    group_model.variables_offset = 0;
    if (group_model.are_variables_contiguous && !robot_var_names.empty()) {
        group_model.variables_offset = std::distance(
                robot_model.getVariableNames().begin(),
                std::find(
                        robot_model.getVariableNames().begin(),
                        robot_model.getVariableNames().end(),
                        robot_var_names.front()));
    }

    ROS_DEBUG_NAMED(CDP_LOGGER, "Sorted Variable Names: %s", to_string(group_model.variable_names).c_str());
    ROS_DEBUG_NAMED(CDP_LOGGER, "Sorted Variable Indices: %s", to_string(group_model.variable_indices).c_str());
    ROS_DEBUG_NAMED(CDP_LOGGER, "Contiguous: %s", group_model.are_variables_contiguous ? "true" : "false");
    ROS_DEBUG_NAMED(CDP_LOGGER, "Variables Offset: %d", group_model.variables_offset);
}

void CollisionWorldSBPL::registerWorldCallback()
{
    ROS_DEBUG("Registering world observer callback");
    auto ocfn = boost::bind(&CollisionWorldSBPL::worldUpdate, this, _1, _2);
    m_observer_handle = getWorld()->addObserver(ocfn);
}

void CollisionWorldSBPL::worldUpdate(
    const World::ObjectConstPtr& object,
    World::Action action)
{
    ROS_INFO("CollisionWorldSBPL::worldUpdate()");
    ROS_INFO("  id: %s", object->id_.c_str());
    ROS_INFO("  shapes: %zu", object->shapes_.size());
    ROS_INFO("  shape_poses: %zu", object->shape_poses_.size());
    if (action & World::ActionBits::UNINITIALIZED) {
        ROS_INFO("  action: UNINITIALIZED");
        processWorldUpdateUninitialized(object);
    }
    else if (action & World::ActionBits::CREATE) {
        ROS_INFO("  action: CREATE");
        processWorldUpdateCreate(object);
    }
    else if (action & World::ActionBits::DESTROY) {
        ROS_INFO("  action: DESTROY");
        processWorldUpdateDestroy(object);
    }
    else if (action & World::ActionBits::MOVE_SHAPE) {
        ROS_INFO("  action: MOVE_SHAPE");
        processWorldUpdateMoveShape(object);
    }
    else if (action & World::ActionBits::ADD_SHAPE) {
        ROS_INFO("  action: ADD_SHAPE");
        processWorldUpdateAddShape(object);
    }
    else if (action & World::ActionBits::REMOVE_SHAPE)  {
        ROS_INFO("  action: REMOVE_SHAPE");
        processWorldUpdateRemoveShape(object);
    }
}

bool CollisionWorldSBPL::checkDegenerateCollision(CollisionResult& res) const
{
    if (!getWorld()) {
        ROS_ERROR_ONCE("Degenerate Collision: No World");
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
        ROS_DEBUG("%zu shapes in object", num_shapes);
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
        ROS_DEBUG("Adding object '%s' to the configuration space", oit->first.c_str());
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
    ROS_ERROR("checkRobotCollision(req, res, robot, state)");
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    const CollisionRobotSBPL& crobot = (const CollisionRobotSBPL&)robot;
    if (state.getRobotModel()->getName() !=
        crobot.robotCollisionModel()->name())
    {
        ROS_ERROR("Collision Robot Model does not match Robot Model");
        setVacuousCollision(res);
        return;
    }

    auto gm = getGroupModel(crobot, *state.getRobotModel(), req.group_name);
    if (!gm) {
        ROS_ERROR("Failed to get Group Model for robot '%s', group '%s'",
                state.getRobotModel()->getName().c_str(),
                req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto cspace = gm->cspace;
    assert(cspace);

    const Eigen::Affine3d& T_model_robot =
            state.getGlobalLinkTransform(state.getRobotModel()->getRootLink());
    cspace->setWorldToModelTransform(T_model_robot);

    std::vector<double> vars = getCheckedVariables(*gm, state);

    // TODO: It would be nice to not have to set this before every call, as the
    // acm changes infrequently yet here forces reevaluation of cached variables
    // in the cspace.
    //
    // Thoughts:
    //
    // * We could check for equality between the cached allowed collision matrix
    //   and the collision matrix here but this is a relatively expensive set of
    //   string operations
    //
    // * create an overload of isStateValid that takes in an allowed collision
    //   matrix and either remove the cached variables underneath or ignore
    //   them when a complete allowed collision matrix is given. The goal is to
    //   not have to set this every time but retain the ability to optimize away
    //   unnecessary allowed collision entry lookups when the acm changes
    //   infrequently. Another consideration is that we may want to have an
    //   internal allowed collision matrix representation in
    //   sbpl_collisiion_checking (potentially one that is more compact) and so
    //   we may want to write a wrapper interface class that gets passed to that
    //   variant of isStateValid that can accept either this representation or
    //   the internal one
    cspace->setAllowedCollisionMatrix(acm);

    double dist;
    const bool verbose = req.verbose;
    const bool visualize = true; //req.verbose;
    bool valid = cspace->isStateValid(vars, verbose, visualize, dist);
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
    ROS_ERROR("checkRobotCollision(req, res, robot, state1, state2)");
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
    ROS_ERROR("checkRobotCollision(req, res, robot, state1, state2, acm)");
}

void CollisionWorldSBPL::processWorldUpdateUninitialized(
    const World::ObjectConstPtr& object)
{

}

void CollisionWorldSBPL::processWorldUpdateCreate(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_models) {
        entry.second->cspace->insertObject(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateDestroy(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_models) {
        entry.second->cspace->removeObject(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateMoveShape(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_models) {
        entry.second->cspace->moveShapes(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateAddShape(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_models) {
        entry.second->cspace->insertShapes(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateRemoveShape(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_models) {
        entry.second->cspace->removeShapes(object);
    }
}

bool CollisionWorldSBPL::worldObjectToCollisionObjectMsgFull(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object) const
{
    moveit_msgs::CollisionObject obj_msg;

    obj_msg.header.stamp = ros::Time(0);

    // TODO: safe to assume that the world frame will always be the world
    // frame or should we try to transform things here somehow
    obj_msg.header.frame_id = m_world_collision_model_config.world_frame;

    obj_msg.id = object.id_;

    assert(object.shape_poses_.size() == object.shapes_.size());
    for (size_t sind = 0; sind < object.shapes_.size(); ++sind) {
        const Eigen::Affine3d& shape_transform = object.shape_poses_[sind];
        const shapes::ShapeConstPtr& shape = object.shapes_[sind];

        // convert shape to corresponding shape_msgs type
        switch (shape->type) {
        case shapes::UNKNOWN_SHAPE:
        {
            ROS_WARN("Object '%s' contains shape of unknown type", object.id_.c_str());
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
            ROS_ERROR("Unsupported object type: Cone");
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
            ROS_ERROR("Unsupported object type: Plane");
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
            ROS_ERROR("Unsupported object type: OcTree");
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
    obj_msg.header.frame_id = m_world_collision_model_config.world_frame;
    obj_msg.id = object.id_;
    collision_object = std::move(obj_msg);
    return true;
}

} // collision_detection
