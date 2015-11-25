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
    // TODO: implement
    ROS_INFO("CollisionWorldSBPL()");
    construct();
}

CollisionWorldSBPL::CollisionWorldSBPL(const WorldPtr& world) :
    CollisionWorld(world)
{
    // TODO: implement
    ROS_INFO("CollisionWorldSBPL(const WorldPtr&)");
    construct();
    registerWorldCallback();
}

CollisionWorldSBPL::CollisionWorldSBPL(
    const CollisionWorldSBPL& other,
    const WorldPtr& world)
:
    CollisionWorld(other, world)
{
    // TODO: implement
    ROS_INFO("CollisionWorldSBPL(CollisionWorldSBPL&, const WorldPtr&)");
    construct();
    registerWorldCallback();
}

CollisionWorldSBPL::~CollisionWorldSBPL()
{
    ROS_INFO("~CollisionWorldSBPL");
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
//    ROS_INFO("checkRobotCollision(req, res, robot, state, acm)");

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
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state) const
{
    ROS_INFO("distanceRobot(robot, state)");
    // TODO: implement
    return -1.0;
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    ROS_INFO("distanceRobot(robot, state, acm)");
    // TODO: implement
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(const CollisionWorld& world) const
{
    ROS_INFO("distanceWorld(world)");
    // TODO: implement
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(
    const CollisionWorld& world,
    const AllowedCollisionMatrix& acm) const
{
    ROS_INFO("distanceWrold(world, acm)");
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
    const sbpl_interface::MoveItRobotModel* sbpl_robot_model,
    const CollisionWorldConfig& collision_world_config,
    const std::string& urdf_string,
    const std::string& group_name,
    const sbpl::collision::CollisionModelConfig& config)
{
    ROS_INFO("Initializing Collision World SBPL");

    if (!m_urdf_string.empty()) {
        ROS_WARN("Collision World SBPL cannot be reinitialized");
        return false;
    }

    WorldPtr curr_world = getWorld();
    if (!curr_world) {
        ROS_WARN("Collision World SBPL requires a World before initalization");
        return false;
    }

    // create distance field large enough to hold world
    ROS_INFO("  Computing world bounding box");
    moveit_msgs::OrientedBoundingBox world_bb = computeWorldAABB(*curr_world);
    ROS_INFO("  -> Bounding Box:");
    ROS_INFO("     position: (%0.3f, %0.3f, %0.3f)", world_bb.pose.position.x, world_bb.pose.position.y, world_bb.pose.position.z);
    ROS_INFO("     extents: (%0.3f, %0.3f, %0.3f)", world_bb.extents.x, world_bb.extents.y, world_bb.extents.z);

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

    const double df_size_x = collision_world_config.size_x; //world_bb.extents.x;
    const double df_size_y = collision_world_config.size_y; //world_bb.extents.y;
    const double df_size_z = collision_world_config.size_z; //world_bb.extents.z;
    const double df_res_m = collision_world_config.res_m; //0.02; // TODO: non-hardcode
    const double df_origin_x = collision_world_config.origin_x; //world_bb.pose.position.x;
    const double df_origin_y = collision_world_config.origin_y; //world_bb.pose.position.y;
    const double df_origin_z = collision_world_config.origin_z; //world_bb.pose.position.z;

    ROS_INFO("  Initializing Distance Field");
    ROS_INFO("    size: (%0.3f, %0.3f, %0.3f)", df_size_x, df_size_y, df_size_z);
    ROS_INFO("    origin: (%0.3f, %0.3f, %0.3f)", df_origin_x, df_origin_y, df_origin_z);
    m_dfield.reset(new distance_field::PropagationDistanceField(
            df_size_x, df_size_y, df_size_z,
            df_res_m,
            df_origin_x, df_origin_y, df_origin_z,
            df_max_distance,
            false));
    ROS_INFO("  Constructing Occupancy Grid");
    m_grid.reset(new sbpl_arm_planner::OccupancyGrid(m_dfield.get()));
    m_grid->setReferenceFrame(collision_world_config.world_frame);
    ROS_INFO("  Constructing Collision Space");
    m_cspace.reset(new sbpl::collision::SBPLCollisionSpace(m_grid.get()));

    ROS_INFO("  Initializing Collision Space");
    if (!m_cspace->init(
            urdf_string,
            group_name,
            config,
            sbpl_robot_model->planningVariableNames()))
    {
        ROS_ERROR("  Failed to initialize collision space");
        // reset data structures from above
        m_cspace.reset();
        m_grid.reset();
        m_dfield.reset();
        return false;
    }

    ROS_INFO("  Successfully initialized collision space");

    // Save these parameters for reinitialization if the world changes.
    // NOTE: must cache these parameters before adding the world to the
    // collision space
    m_sbpl_robot_model = sbpl_robot_model;
    m_urdf_string = urdf_string;
    m_group_name = group_name;
    m_cm_config = config;
    m_cw_config = collision_world_config;

    ROS_INFO("  Adding world to collision space");
    addWorldToCollisionSpace(*curr_world);
    ROS_INFO("  Added world to collision space");

    // publish collision world visualizations
    ROS_INFO("Publishing visualization of bounding box");
    visualization_msgs::MarkerArray markers;
    markers = m_cspace->getBoundingBoxVisualization();
    m_cspace_pub.publish(markers);

    return true;
}

void CollisionWorldSBPL::construct()
{
    m_sbpl_robot_model = nullptr;
    m_cspace_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "collision_space", 5);
}

bool CollisionWorldSBPL::initialized() const
{
    // if no world, collisions are vacuously empty, otherwise initialization is
    // required to determine collisions
    return (!getWorld() || m_cspace);
}

const distance_field::PropagationDistanceField*
CollisionWorldSBPL::distanceField() const
{
    return m_dfield.get();
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
        processWorldUpdateUninitialized(*object);
    }
    else if (action & World::ActionBits::CREATE) {
        ROS_INFO("  action: CREATE");
        processWorldUpdateCreate(*object);
    }
    else if (action & World::ActionBits::DESTROY) {
        ROS_INFO("  action: DESTROY");
        processWorldUpdateDestroy(*object);
    }
    else if (action & World::ActionBits::MOVE_SHAPE) {
        ROS_INFO("  action: MOVE_SHAPE");
        processWorldUpdateMoveShape(*object);
    }
    else if (action & World::ActionBits::ADD_SHAPE) {
        ROS_INFO("  action: ADD_SHAPE");
        processWorldUpdateAddShape(*object);
    }
    else if (action & World::ActionBits::REMOVE_SHAPE)  {
        ROS_INFO("  action: REMOVE_SHAPE");
        processWorldUpdateRemoveShape(*object);
    }
}

bool CollisionWorldSBPL::checkDegenerateCollision(CollisionResult& res) const
{
    if (!initialized()) {
        ROS_ERROR_ONCE("Degenerate Collision: Uninitialized Collision World SBPL");
        setVacuousCollision(res);
        return true;
    }

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

void CollisionWorldSBPL::addWorldToCollisionSpace(const World& world)
{
    for (auto oit = world.begin(); oit != world.end(); ++oit) {
        ROS_DEBUG("Adding object '%s' to the configuration space", oit->first.c_str());
        assert(oit->second.get());

        const std::string& name = oit->first;
        const World::Object& object = *oit->second;

        // convert to World::Object to moveit_msgs::CollisionObject
        moveit_msgs::CollisionObject obj_msg;
        if (!worldObjectToCollisionObjectMsgFull(object, obj_msg)) {
            ROS_WARN("Failed to convert world object '%s' to collision object", object.id_.c_str());
            continue;
        }
        obj_msg.operation = moveit_msgs::CollisionObject::ADD;
        m_cspace->processCollisionObject(obj_msg);
    }
}

void CollisionWorldSBPL::updateCollisionSpaceJointState(
    const moveit::core::RobotState& state)
{
    // first update => need to update everything
    if (m_updated_joint_variables.size() != state.getVariableCount()) {
        ROS_INFO("Initializing collision space joint state");

        m_updated_joint_variables.resize(state.getVariableCount());
        for (size_t vind = 0; vind < state.getVariableCount(); ++vind) {
            const std::string& variable_name = state.getVariableNames()[vind];
            double variable_position = state.getVariablePositions()[vind];
            m_updated_joint_variables[vind] = variable_position;
            ROS_DEBUG("Syncing joint variable '%s'", variable_name.c_str());
            m_cspace->setJointPosition(variable_name, variable_position);
        }

        m_cspace->updateVoxelGroups();

        ROS_INFO("Publishing visualization of occupied_voxels");
        visualization_msgs::MarkerArray markers;
        markers = m_cspace->getOccupiedVoxelsVisualization();
        m_cspace_pub.publish(markers);

        ros::Duration(1.0).sleep();

//        std::vector<double> pvars = extractPlanningVariables(state);
//        markers = m_cspace->getCollisionModelVisualization(pvars);
//        m_cspace_pub.publish(markers);

        ROS_INFO("Collision space joint state initialized");
        return;
    }

    // update only joint variables that have changed
    assert(m_updated_joint_variables.size() == state.getVariableCount());
    for (size_t vind = 0; vind < state.getVariableCount(); ++vind) {
        double variable_position = state.getVariablePositions()[vind];
        if (m_updated_joint_variables[vind] != variable_position) {
            const std::string& variable_name = state.getVariableNames()[vind];
            m_updated_joint_variables[vind];
            m_cspace->setJointPosition(variable_name, variable_position);
        }
    }
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state)
{
    ROS_ERROR("checkRobotCollision(req, res, robot, state)");
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    updateCollisionSpaceJointState(state);

    std::vector<double> pvars = extractPlanningVariables(state);

    double dist;
    bool check_res = m_cspace->isStateValid(pvars, req.verbose, req.verbose, dist);
    if (req.verbose) {
        auto markers = m_cspace->getCollisionModelVisualization(pvars);
        m_cspace_pub.publish(markers);
    }

    res.collision = !check_res;
    if (req.distance) {
        res.distance = dist;
    }
    if (req.cost) {
        ROS_ERROR("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_ERROR("Contacts not computed by sbpl collision checker");
    }
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2)
{
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
    ROS_ERROR("checkRobotCollision(req, res, robot, state1, state2, acm)");
}

std::vector<double> CollisionWorldSBPL::extractPlanningVariables(
    const moveit::core::RobotState& state) const
{
    assert(m_sbpl_robot_model);
    int av_count = m_sbpl_robot_model->activeVariableCount();
    std::vector<double> pvars(av_count);
    for (int avind = 0; avind < av_count; ++avind) {
        int vind = m_sbpl_robot_model->activeVariableIndices()[avind];
        pvars[avind] = state.getVariablePositions()[vind];
    }
    return pvars;
}

void CollisionWorldSBPL::processWorldUpdateUninitialized(
    const World::Object& object)
{

}

void CollisionWorldSBPL::processWorldUpdateCreate(
    const World::Object& object)
{
    if (!m_cspace) {
        ROS_ERROR("Collision Space has not been initialized");
        return;
    }

    // convert to collision object
    moveit_msgs::CollisionObject collision_object;
    if (!worldObjectToCollisionObjectMsgFull(object, collision_object)) {
        ROS_ERROR("Failed to convert world object '%s' to collision object", object.id_.c_str());
        return;
    }
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    m_cspace->processCollisionObject(collision_object);
}

void CollisionWorldSBPL::processWorldUpdateDestroy(
    const World::Object& object)
{
    moveit_msgs::CollisionObject collision_object;
    if (!worldObjectToCollisionObjectMsgName(object, collision_object)) {
        ROS_ERROR("Failed to convert world object '%s' to collision object", object.id_.c_str());
        return;
    }
    collision_object.operation = moveit_msgs::CollisionObject::REMOVE;

    m_cspace->processCollisionObject(collision_object);
}

void CollisionWorldSBPL::processWorldUpdateMoveShape(
    const World::Object& object)
{

}

void CollisionWorldSBPL::processWorldUpdateAddShape(
    const World::Object& object)
{

}

void CollisionWorldSBPL::processWorldUpdateRemoveShape(
    const World::Object& object)
{

}

bool CollisionWorldSBPL::worldObjectToCollisionObjectMsgFull(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object) const
{
    moveit_msgs::CollisionObject obj_msg;

    obj_msg.header.stamp = ros::Time(0);

    // TODO: safe to assume that the world frame will always be the world
    // frame or should we try to transform things here somehow
    obj_msg.header.frame_id = m_cw_config.world_frame;

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
            ROS_ERROR("Unsupported object type: Mesh");
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
    obj_msg.header.frame_id = m_cw_config.world_frame;
    obj_msg.id = object.id_;
    collision_object = std::move(obj_msg);
    return true;
}

} // collision_detection
