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

// system includes
#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>
#include <leatherman/print.h>
#include <eigen_conversions/eigen_msg.h>

// project includes
#include <moveit_planners_sbpl/moveit_robot_model.h>

namespace collision_detection {

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
    ROS_INFO("CollisionWorldSBPL(CollisionWorldSBPL&, const WorldPtr&)");

    m_world_collision_model_config = other.m_world_collision_model_config;
    m_robot_collision_model_config = other.m_robot_collision_model_config;
    m_group_to_collision_space = other.m_group_to_collision_space;
    m_group_to_grid = other.m_group_to_grid;
    // NOTE: no need to copy observer handle
    registerWorldCallback();
    // NOTE:no need to copy node handle
    m_cspace_pub = other.m_cspace_pub;
}

CollisionWorldSBPL::~CollisionWorldSBPL()
{
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
    ros::NodeHandle nh;

    // load occupancy grid parameters
    XmlRpc::XmlRpcValue world_collision_model_cfg;
    if (nh.getParam("world_collision_model", world_collision_model_cfg)) {
        // TODO: more sophisticated parameter checking
        m_world_collision_model_config.world_frame = "world"; //scene.getPlanningFrame();
        m_world_collision_model_config.size_x =
                world_collision_model_cfg["size_x"];
        m_world_collision_model_config.size_y =
                world_collision_model_cfg["size_y"];
        m_world_collision_model_config.size_z =
                world_collision_model_cfg["size_z"];
        m_world_collision_model_config.origin_x =
                world_collision_model_cfg["origin_x"];
        m_world_collision_model_config.origin_y =
                world_collision_model_cfg["origin_y"];
        m_world_collision_model_config.origin_z =
                world_collision_model_cfg["origin_z"];
        m_world_collision_model_config.res_m =
                world_collision_model_cfg["res_m"];
        m_world_collision_model_config.max_distance_m =
                world_collision_model_cfg["max_distance_m"];
    }
    else {
        // TODO: reasonable defaults or just toss out an exception?
        ROS_ERROR("Failed to retrieve 'collision_world' from the param server");
    }

    // load collision model parameters
    if (!sbpl::collision::CollisionModelConfig::Load(
            nh, m_robot_collision_model_config))
    {
        ROS_ERROR("Failed to load Collision Model Config");
    }

    m_cspace_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);
}

sbpl::collision::CollisionSpacePtr CollisionWorldSBPL::getCollisionSpace(
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name)
{
    auto it = m_group_to_collision_space.find(group_name);
    if (it != m_group_to_collision_space.end()) {
        return it->second;
    }

    ROS_INFO("Creating Collision Space for group '%s'", group_name.c_str());

    auto urdf = robot_model.getURDF();
    if (!urdf) {
        ROS_ERROR("URDF not found in Robot Model");
        return sbpl::collision::CollisionSpacePtr();
    }

    // TODO: should be dependent on size of the largest sphere in the collision
    // group model (don't forget about attached objects, way to set this
    // explicitly on the collision space or the occupancy grid?)
    const double df_max_distance = 0.2;

    const double df_size_x = m_world_collision_model_config.size_x;
    const double df_size_y = m_world_collision_model_config.size_y;
    const double df_size_z = m_world_collision_model_config.size_z;
    const double df_res_m = m_world_collision_model_config.res_m;
    const double df_origin_x = m_world_collision_model_config.origin_x;
    const double df_origin_y = m_world_collision_model_config.origin_y;
    const double df_origin_z = m_world_collision_model_config.origin_z;

    ROS_INFO("  Initializing Distance Field");
    ROS_INFO("    size: (%0.3f, %0.3f, %0.3f)", df_size_x, df_size_y, df_size_z);
    ROS_INFO("    origin: (%0.3f, %0.3f, %0.3f)", df_origin_x, df_origin_y, df_origin_z);

    auto df = std::make_shared<distance_field::PropagationDistanceField>(
            df_size_x, df_size_y, df_size_z,
            df_res_m,
            df_origin_x, df_origin_y, df_origin_z,
            df_max_distance,
            false);

    ROS_INFO("  Constructing Occupancy Grid");

    // TODO: semi-annoying to have to keep this around, but is it worth the
    // refactoring to convert all raw pointers to occupancy grids to their
    // shared variants?
    auto grid = std::make_shared<sbpl::OccupancyGrid>(df);

    grid->setReferenceFrame(m_world_collision_model_config.world_frame);

    ROS_INFO("  Constructing Collision Space");
    auto cspace = std::make_shared<sbpl::collision::CollisionSpace>(
            grid.get());

    // we're just going to send off all robot state variables to isStateValid
    // calls since there isn't a notion of planning variables here and the
    // collision model should be maintaining state for all these joint variables
    const auto& checked_variables = robot_model.getVariableNames();

    ROS_INFO("  Initializing Collision Space");
    if (!cspace->init(
            *urdf,
            group_name,
            m_robot_collision_model_config,
            checked_variables))
    {
        ROS_ERROR("  Failed to initialize collision space");
        return sbpl::collision::CollisionSpacePtr();
    }

    ROS_INFO("  Successfully initialized collision space");

    ROS_INFO("  Adding world to collision space");
    WorldPtr curr_world = getWorld();
    if (curr_world) {
        // TODO: originally the intention was to automatically determine the
        // distance field boundaries inherited from the world bounding box.
        // Currently this doesn't seem like the thing to do anymore, since the
        // distance field extents are also considered boundaries for collision
        // checking. Warrants review, since the workspace boundaries in the
        // planning request seem like the proper way to express boundaries for
        // the robot
        ROS_INFO("  Computing world bounding box");
        moveit_msgs::OrientedBoundingBox world_bb = computeWorldAABB(*curr_world);
        ROS_INFO("  -> Bounding Box:");
        ROS_INFO("     position: (%0.3f, %0.3f, %0.3f)", world_bb.pose.position.x, world_bb.pose.position.y, world_bb.pose.position.z);
        ROS_INFO("     extents: (%0.3f, %0.3f, %0.3f)", world_bb.extents.x, world_bb.extents.y, world_bb.extents.z);

        addWorldToCollisionSpace(*curr_world, *cspace);
    }
    ROS_INFO("  Added world to collision space");

    // publish collision world visualizations
    ROS_INFO("Publishing visualization of bounding box");
    visualization_msgs::MarkerArray markers;
    markers = cspace->getBoundingBoxVisualization();
    m_cspace_pub.publish(markers);

    m_group_to_grid[group_name] = grid;
    m_group_to_collision_space[group_name] = cspace;

    return cspace;
}

const distance_field::PropagationDistanceField*
CollisionWorldSBPL::distanceField(const std::string& group_name) const
{
    auto it = m_group_to_grid.find(group_name);
    if (it == m_group_to_grid.end()) {
        return nullptr;
    }
    else {
        return it->second->getDistanceField().get();
    }
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
    auto cspace = getCollisionSpace(*state.getRobotModel(), req.group_name);
    if (!cspace) {
        ROS_ERROR("Failed to get Collision Space for group '%s'", req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    std::vector<double> vars(
            state.getVariablePositions(),
            state.getVariablePositions() + state.getVariableCount());

    double dist;
    const bool verbose = req.verbose;
    const bool visualize = true; //req.verbose;
    bool check_res = cspace->isStateValid(vars, verbose, visualize, dist);
    if (visualize) {
//        auto markers = cspace->getCollisionModelVisualization(vars);
        auto markers = cspace->getVisualization("collision_model");
        m_cspace_pub.publish(markers);
    }

    res.collision = !check_res;
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
    for (const auto& entry : m_group_to_collision_space) {
        entry.second->insertObject(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateDestroy(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_to_collision_space) {
        entry.second->removeObject(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateMoveShape(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_to_collision_space) {
        entry.second->moveShapes(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateAddShape(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_to_collision_space) {
        entry.second->insertShapes(object);
    }
}

void CollisionWorldSBPL::processWorldUpdateRemoveShape(
    const World::ObjectConstPtr& object)
{
    for (const auto& entry : m_group_to_collision_space) {
        entry.second->removeShapes(object);
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
