////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#include "collision_common_sbpl.h"

// standard includes
#include <sstream>
#include <stdexcept>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <smpl/console/nonstd.h>

namespace collision_detection {

CollisionStateUpdater::CollisionStateUpdater() :
    m_rcm_var_indices(),
    m_rcm_vars(),
    m_rcs(),
    m_inorder(false)
{
}

bool CollisionStateUpdater::init(
    const moveit::core::RobotModel& robot,
    const sbpl::collision::RobotCollisionModelConstPtr& rcm)
{
    using sbpl::collision::RobotCollisionState;
    using sbpl::collision::AttachedBodiesCollisionModel;
    using sbpl::collision::AttachedBodiesCollisionState;

    if (!getRobotCollisionModelJointIndices(
            robot.getVariableNames(), *rcm, m_rcm_var_indices))
    {
        return false;
    }

    m_rcs = std::make_shared<RobotCollisionState>(rcm.get());
    m_ab_model = std::make_shared<AttachedBodiesCollisionModel>(rcm.get());
    m_ab_state = std::make_shared<AttachedBodiesCollisionState>(
            m_ab_model.get(), m_rcs.get());

    m_rcm_vars.assign(robot.getVariableCount(), 0.0);

    m_inorder = true;
    for (size_t i = 1; i < m_rcm_var_indices.size(); ++i) {
        if (m_rcm_var_indices[i] != m_rcm_var_indices[i - 1] + 1) {
            m_inorder = false;
            ROS_INFO("Joint variables not in order:");
            ROS_INFO_STREAM("  RobotModel: " << robot.getVariableNames());
            ROS_INFO_STREAM("  RobotCollisionModel: " << rcm->jointVarNames());
            break;
        }
    }
    ROS_INFO("Ordered: %s", m_inorder ? "true" : "false");
    return true;
}

void CollisionStateUpdater::update(const moveit::core::RobotState& state)
{
    m_rcs->setJointVarPositions(getVariablesFor(state).data());
    updateAttachedBodies(state);
}

const std::vector<double>& CollisionStateUpdater::getVariablesFor(
    const moveit::core::RobotState& state)
{
    if (m_inorder) {
        std::copy(
                state.getVariablePositions(),
                state.getVariablePositions() + state.getVariableCount(),
                m_rcm_vars.begin());
    } else {
        for (size_t vidx = 0; vidx < state.getVariableCount(); ++vidx) {
            int rcmvidx = m_rcm_var_indices[vidx];
            m_rcm_vars[rcmvidx] = state.getVariablePosition(vidx);
        }
    }
    return m_rcm_vars;
}

bool CollisionStateUpdater::getRobotCollisionModelJointIndices(
    const std::vector<std::string>& joint_names,
    const sbpl::collision::RobotCollisionModel& rcm,
    std::vector<int>& rcm_joint_indices)
{
    // check for joint existence
    for (const std::string& joint_name : joint_names) {
        if (!rcm.hasJointVar(joint_name)) {
            ROS_ERROR("Joint variable '%s' not found in Robot Collision Model", joint_name.c_str());
            return false;
        }
    }

    // map planning joint indices to collision model indices
    rcm_joint_indices.resize(joint_names.size(), -1);

    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];;
        int jidx = rcm.jointVarIndex(joint_name);

        rcm_joint_indices[i] = jidx;
    }

    return true;
}

bool CollisionStateUpdater::updateAttachedBodies(
    const moveit::core::RobotState& state)
{
    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    state.getAttachedBodies(attached_bodies);

    bool updated = false;

    // add bodies not in the attached body model
    for (const moveit::core::AttachedBody* ab : attached_bodies) {
        if (!m_ab_model->hasAttachedBody(ab->getName())) {
            updated = true;
            ROS_DEBUG("Attach body '%s' from Robot Collision Model", ab->getName().c_str());
            m_ab_model->attachBody(ab->getName(), ab->getShapes(), ab->getFixedTransforms(), ab->getAttachedLinkName());
            for (const auto& touch_link : ab->getTouchLinks()) {
                m_touch_link_map.insert(std::make_pair(ab->getName(), touch_link));
                m_touch_link_map.insert(std::make_pair(touch_link, ab->getName()));
            }
        }
    }

    // remove bodies not in the list of attached bodies
    if (m_ab_model->attachedBodyCount() > attached_bodies.size()) {
        updated = true;
        std::vector<int> abindices;
        m_ab_model->attachedBodyIndices(abindices);
        for (int abidx : abindices) {
            const std::string& ab_name = m_ab_model->attachedBodyName(abidx);
            auto it = std::find_if(
                    attached_bodies.begin(), attached_bodies.end(),
                    [&ab_name](const moveit::core::AttachedBody* ab)
                    {
                        return ab->getName() == ab_name;
                    });
            if (it == attached_bodies.end()) {
                ROS_DEBUG("Detach body '%s' from Robot Collision Model", ab_name.c_str());
                m_ab_model->detachBody(ab_name);
                auto it = m_touch_link_map.begin();
                while (it != m_touch_link_map.end()) {
                    if (it->first == ab_name || it->second == ab_name) {
                        it = m_touch_link_map.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        }
    }

    return updated;
}

void LoadCollisionGridConfig(
    ros::NodeHandle& nh,
    const std::string& param_name,
    CollisionGridConfig& config)
{
    // resolve param
    std::string cm_key;
    if (!nh.searchParam(param_name, cm_key)) {
        std::stringstream ss;
        ss << "Failed to find '" << param_name << "' key on the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    ROS_INFO("Found '%s' param at %s", param_name.c_str(), cm_key.c_str());

    // read parameter
    XmlRpc::XmlRpcValue cm_config;
    if (!nh.getParam(cm_key, cm_config)) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << cm_key << "' from the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    std::vector<std::string> required_fields =
    {
        "size_x",
        "size_y",
        "size_z",
        "origin_x",
        "origin_y",
        "origin_z",
        "res_m",
        "max_distance_m",
    };

    // check for required type and members
    if (cm_config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !std::all_of(required_fields.begin(), required_fields.end(),
                [&](const std::string& field)
                { return cm_config.hasMember(field); }))
    {
        std::stringstream ss;
        ss << "'" << param_name << "' param is malformed";
        ROS_ERROR_STREAM(ss.str());
        for (const auto& field : required_fields) {
            ROS_ERROR_STREAM("has " << field << " member: " << std::boolalpha << cm_config.hasMember(field));
        }
        throw std::runtime_error(ss.str());
    }

    if (cm_config.hasMember("frame_id")) {
        config.frame_id = (std::string)cm_config["frame_id"];
    } else {
        config.frame_id = "map";
    }

    // TODO: more sophisticated parameter checking
    config.size_x = cm_config["size_x"];
    config.size_y = cm_config["size_y"];
    config.size_z = cm_config["size_z"];
    config.origin_x = cm_config["origin_x"];
    config.origin_y = cm_config["origin_y"];
    config.origin_z = cm_config["origin_z"];
    config.res_m = cm_config["res_m"];
    config.max_distance_m = cm_config["max_distance_m"];
}

/// \brief Load the Joint <-> Collision Group Map from the param server
///
/// If the expected parameter is not found or the value on the param server is
/// malformed, the group map is left unaltered; otherwise it is overwritten.
bool LoadJointCollisionGroupMap(
    ros::NodeHandle& nh,
    std::unordered_map<std::string, std::string>& _jcgm_map)
{
    const char* jcgm_param = "joint_collision_group_map";
    std::string jcgm_key;
    if (!nh.searchParam(jcgm_param, jcgm_key)) {
        ROS_INFO("No param '%s' found on the param server. assuming same names for joint and collision groups", jcgm_param);
        return false;
    }

    XmlRpc::XmlRpcValue jcgm_value;
    if (!nh.getParam(jcgm_key, jcgm_value)) {
        ROS_ERROR("Failed to retrieve param '%s' from the param server?", jcgm_key.c_str());
        return false;
    }

    if (jcgm_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("'%s' value must be a struct", jcgm_param);
        return false;
    }

    std::unordered_map<std::string, std::string> jcgm_map;
    for (auto it = jcgm_value.begin(); it != jcgm_value.end(); ++it) {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("'%s' dictionary elements must be strings", jcgm_param);
            return false;
        }

        jcgm_map[it->first] = (std::string)it->second;
    }

    _jcgm_map = std::move(jcgm_map);
    return true;
}

// Converts a world object to a collision object.
// The collision object's frame_id is the planning frame and the operation
// is unspecified via this call
bool WorldObjectToCollisionObjectMsgFull(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object)
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

bool WorldObjectToCollisionObjectMsgName(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object)
{
    moveit_msgs::CollisionObject obj_msg;
    obj_msg.header.stamp = ros::Time(0);
//    obj_msg.header.frame_id = m_wcm_config.world_frame;
    obj_msg.id = object.id_;
    collision_object = std::move(obj_msg);
    return true;
}

// For primitive shapes, just copy to the corresponding shape type; for more
// heavyweight shapes, reference the origin's data members.
static auto MakeCollisionShape(const shapes::Shape& shape)
    -> std::unique_ptr<sbpl::collision::CollisionShape>
{
    switch (shape.type) {
    case shapes::ShapeType::UNKNOWN_SHAPE:
        return nullptr;
    case shapes::ShapeType::SPHERE:
    {
        auto& sphere = static_cast<const shapes::Sphere&>(shape);
        return std::unique_ptr<sbpl::collision::SphereShape>(
                new sbpl::collision::SphereShape(sphere.radius));
    }
    case shapes::ShapeType::CYLINDER:
    {
        auto& cylinder = static_cast<const shapes::Cylinder&>(shape);
        return std::unique_ptr<sbpl::collision::CylinderShape>(
                new sbpl::collision::CylinderShape(cylinder.radius, cylinder.length));
    }
    case shapes::ShapeType::CONE:
    {
        auto& cone = static_cast<const shapes::Cone&>(shape);
        return std::unique_ptr<sbpl::collision::ConeShape>(
                new sbpl::collision::ConeShape(cone.radius, cone.length));
    }
    case shapes::ShapeType::BOX:
    {
        auto& box = static_cast<const shapes::Box&>(shape);
        return std::unique_ptr<sbpl::collision::BoxShape>(
                new sbpl::collision::BoxShape(box.size[0], box.size[1], box.size[2]));
    }
    case shapes::ShapeType::PLANE:
    {
        auto& plane = static_cast<const shapes::Plane&>(shape);
        return std::unique_ptr<sbpl::collision::PlaneShape>(
                new sbpl::collision::PlaneShape(plane.a, plane.b, plane.c, plane.d));
    }
    case shapes::ShapeType::MESH:
    {
        auto& mesh = static_cast<const shapes::Mesh&>(shape);
        auto imesh = std::unique_ptr<sbpl::collision::MeshShape>(
                new sbpl::collision::MeshShape);
        imesh->vertices = mesh.vertices;
        imesh->vertex_count = mesh.vertex_count;
        imesh->triangles = mesh.triangles; // hopefully this cast is well-formed
        imesh->triangle_count = mesh.triangle_count;
        return std::move(imesh);
    }
    case shapes::ShapeType::OCTREE:
    {
        auto& octree = static_cast<const shapes::OcTree&>(shape);
        auto ioctree = std::unique_ptr<sbpl::collision::OcTreeShape>(
                new sbpl::collision::OcTreeShape);
        ioctree->octree = octree.octree.get();
        return std::move(ioctree);
    }
    }
}

void ConvertObjectToCollisionObjectShallow(
    const World::ObjectConstPtr& o,
    std::vector<std::unique_ptr<sbpl::collision::CollisionShape>>& collision_shapes,
    std::unique_ptr<sbpl::collision::CollisionObject>& collision_object)
{
    // create uniquely owned, corresponding shapes and gather to connect to
    // CollisionObject
    std::vector<sbpl::collision::CollisionShape*> shapes;
    for (auto& shape_ : o->shapes_) {
        auto shape = MakeCollisionShape(*shape_);
        shapes.push_back(shape.get());
        collision_shapes.push_back(std::move(shape));
    }

    // copy shape poses
    sbpl::collision::AlignedVector<Eigen::Affine3d> shape_poses;
    shape_poses.reserve(o->shape_poses_.size());
    for (auto& shape_pose : o->shape_poses_) {
        shape_poses.push_back(shape_pose);
    }

    // create CollisionObject, deliver the goods
    collision_object = std::unique_ptr<sbpl::collision::CollisionObject>(new sbpl::collision::CollisionObject);
    collision_object->id = o->id_;
    collision_object->shapes = std::move(shapes);
    collision_object->shape_poses = std::move(shape_poses);
}

auto GetCollisionMarkers(sbpl::collision::RobotCollisionState& rcs)
    -> visualization_msgs::MarkerArray
{
    rcs.updateSphereStates();
    return rcs.getVisualization();
}

auto GetCollisionMarkers(sbpl::collision::RobotCollisionState& rcs, int gidx)
    -> visualization_msgs::MarkerArray
{
    // update the spheres within the group
    for (int ssidx : rcs.groupSpheresStateIndices(gidx)) {
        rcs.updateSphereStates(ssidx);
    }
    auto ma = rcs.getVisualization(gidx);
    return ma;
}

auto GetCollisionMarkers(
    sbpl::collision::AttachedBodiesCollisionState& abcs,
    int gidx)
    -> visualization_msgs::MarkerArray
{
    for (int ssidx : abcs.groupSpheresStateIndices(gidx)) {
        abcs.updateSphereStates(ssidx);
    }
    return abcs.getVisualization(gidx);
}

auto GetCollisionMarkers(
    sbpl::collision::RobotCollisionState& rcs,
    sbpl::collision::AttachedBodiesCollisionState& abcs,
    int gidx)
    -> visualization_msgs::MarkerArray
{
    auto ma = GetCollisionMarkers(rcs, gidx);
    auto abma = GetCollisionMarkers(abcs, gidx);
    ma.markers.insert(ma.markers.end(), abma.markers.begin(), abma.markers.end());

    int id = 0;
    for (auto& m : ma.markers) {
        m.id = id++;
    }
    return ma;
}

} // namespace collision_detection
