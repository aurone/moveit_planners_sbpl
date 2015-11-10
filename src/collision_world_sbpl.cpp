#include "collision_world_sbpl.h"

#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>

#include "moveit_robot_model.h"

namespace collision_detection {

CollisionWorldSBPL::CollisionWorldSBPL() :
    CollisionWorld(),
    m_sbpl_robot_model(nullptr),
    m_urdf_string(),
    m_group_name(),
    m_cc_config(),
    m_dfield(),
    m_grid(),
    m_cspace(),
    m_observer_handle(),
    m_updated_joint_variables()
{
    // TODO: implement
    ROS_INFO("CollisionWorldSBPL()");
}

CollisionWorldSBPL::CollisionWorldSBPL(const WorldPtr& world) :
    CollisionWorld(world),
    m_urdf_string(),
    m_group_name(),
    m_cc_config(),
    m_dfield(),
    m_grid(),
    m_cspace(),
    m_observer_handle(),
    m_updated_joint_variables()
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
    m_urdf_string(),
    m_group_name(),
    m_cc_config(),
    m_dfield(),
    m_grid(),
    m_cspace(),
    m_observer_handle(),
    m_updated_joint_variables()
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
    const sbpl_interface::MoveItRobotModel* sbpl_robot_model,
    const CollisionWorldConfig& collision_world_config,
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

    const double df_size_x = collision_world_config.size_x; //world_bb.extents.x;
    const double df_size_y = collision_world_config.size_y; //world_bb.extents.y;
    const double df_size_z = collision_world_config.size_z; //world_bb.extents.z;
    const double df_res_m = collision_world_config.res_m; //0.02; // TODO: non-hardcode
    const double df_origin_x = collision_world_config.origin_x; //world_bb.pose.position.x;
    const double df_origin_y = collision_world_config.origin_y; //world_bb.pose.position.y;
    const double df_origin_z = collision_world_config.origin_z; //world_bb.pose.position.z;

    ROS_INFO("Initializing distance field to size (%0.3f, %0.3f, %0.3f) at (%0.3f, %0.3f, %0.3f)", df_size_x, df_size_y, df_size_z, df_origin_x, df_origin_y, df_origin_z);
    m_dfield.reset(new distance_field::PropagationDistanceField(
            df_size_x, df_size_y, df_size_z,
            df_res_m,
            df_origin_x, df_origin_y, df_origin_z,
            df_max_distance,
            false));
    ROS_INFO("Constructing Occupancy Grid");
    m_grid.reset(new sbpl_arm_planner::OccupancyGrid(m_dfield.get()));
    ROS_INFO("Constructing Collision Space");
    m_cspace.reset(new sbpl::collision::SBPLCollisionSpace(m_grid.get()));

    ROS_INFO("Initializing collision space");
    if (!m_cspace->init(
            urdf_string,
            group_name,
            config,
            sbpl_robot_model->planningVariableNames()))
    {
        ROS_ERROR("Failed to initialize collision space");
        // reset data structures from above
        m_cspace.reset();
        m_grid.reset();
        m_dfield.reset();
        return false;
    }

    ROS_INFO("Successfully initialized collision space");

    // update collision model to the current state of the world

    ROS_INFO("Adding world to collision space");

    addWorldToCollisionSpace(*curr_world);

    ROS_INFO("Added world to collision space");

    // save these parameters for reinitialization if the world changes
    m_sbpl_robot_model = sbpl_robot_model;
    m_urdf_string = urdf_string;
    m_group_name = group_name;
    m_cc_config = config;
    return true;
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

void CollisionWorldSBPL::addWorldToCollisionSpace(const World& world)
{
    for (auto oit = world.begin(); oit != world.end(); ++oit) {
        ROS_INFO("Adding object '%s' to the configuration space", oit->first.c_str());
        assert(oit->second.get());

        const std::string& name = oit->first;
        const World::Object& object = *oit->second;

        // convert to moveit_msgs::CollisionObject
        moveit_msgs::CollisionObject obj_msg;
        obj_msg.header.seq = 0;
        obj_msg.header.stamp = ros::Time(0);
        obj_msg.header.frame_id = "planning_frame";

        assert(object.shape_poses_.size() == object.shapes_.size());
        for (size_t sind = 0; sind < object.shapes_.size(); ++sind) {
            const Eigen::Affine3d& shape_transform = object.shape_poses_[sind];
            const shapes::ShapeConstPtr& shape = object.shapes_[sind];

            switch (shape->type) {
            case shapes::UNKNOWN_SHAPE:
            {
                ROS_WARN("Skipping shape of unknown type");
                continue;
            }
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
            }
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
            }
            case shapes::CONE:
            {

            }
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
            }
            case shapes::PLANE:
            {

            }
            case shapes::MESH:
            {

            }
            case shapes::OCTREE:
            {

            }
            }
        }

        obj_msg.operation = moveit_msgs::CollisionObject::ADD;
        m_cspace->processCollisionObjectMsg(obj_msg);
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
            ROS_INFO("Syncing joint variable '%s'", variable_name.c_str());
            m_cspace->setJointPosition(variable_name, variable_position);
        }

        // set the order of joints
        int av_count = m_sbpl_robot_model->activeVariableCount();
        std::vector<double> planning_variables(av_count);
        for (int avind = 0; avind < av_count; ++avind) {
            int vind = m_sbpl_robot_model->activeVariableIndices()[avind];
            planning_variables[avind] = state.getVariablePositions()[vind];
        }

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

}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    updateCollisionSpaceJointState(state);

    int av_count = m_sbpl_robot_model->activeVariableCount();
    std::vector<double> planning_variables(av_count);
    for (int avind = 0; avind < av_count; ++avind) {
        int vind = m_sbpl_robot_model->activeVariableIndices()[avind];
        planning_variables[avind] = state.getVariablePositions()[vind];
    }

    double dist;
    bool check_res = m_cspace->isStateValid(
            planning_variables, req.verbose, false, dist);

    res.collision = check_res;
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

}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm)
{

}

} // collision_detection
