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

#include <moveit_planners_sbpl/collision_robot_sbpl.h>

#include <ros/ros.h>

namespace collision_detection {

// crp = collision robot plugin
static const char* CRP_LOGGER = "self_collisions";

CollisionRobotSBPL::CollisionRobotSBPL(
    const robot_model::RobotModelConstPtr& model,
    double padding,
    double scale)
:
    CollisionRobot(model, padding, scale)
{
    ROS_INFO_NAMED(CRP_LOGGER, "CollisionRobotSBPL(const RobotModelConstPtr&, double, double)");
    ros::NodeHandle ph("~");

    // search for the robot collision model on the param server
    const char* robot_collision_model_param = "robot_collision_model";
    std::string rcm_key;
    if (!ph.searchParam(robot_collision_model_param, rcm_key)) {
        std::stringstream ss;
        ss << "Failed to find '" << robot_collision_model_param <<
                "' key on the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    // retrieve the robot collision model param from the param server
    XmlRpc::XmlRpcValue rcm_config;
    if (!ph.getParam(rcm_key, rcm_config)) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << rcm_key << "' from the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    // load the robot collision model configuration
    sbpl::collision::CollisionModelConfig cm_config;
    if (!sbpl::collision::CollisionModelConfig::Load(rcm_config, cm_config)) {
        const char* msg = "Failed to load Collision Model Config";
        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
    }

    // build robot collision model from configuration
    auto rcm = sbpl::collision::RobotCollisionModel::Load(
            *model->getURDF(), cm_config);
    if (!rcm) {
        const char* msg = "Failed to build Robot Collision Model from config";
        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
    }

    const char* self_collision_model_param = "self_collision_model";
    LoadCollisionGridConfig(ph, self_collision_model_param, m_scm_config);

    LoadJointCollisionGroupMap(ph, m_jcgm_map);

    if (!m_updater.init(*model, rcm)) {
        const char* msg = "Failed to initialize Collision State Updater";
        ROS_ERROR_NAMED(CRP_LOGGER, "%s", msg);
        throw std::runtime_error(msg);
    }

    // ok! store the robot collision model
    m_rcm = rcm;

    ros::NodeHandle nh;
    m_collision_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);
}

CollisionRobotSBPL::CollisionRobotSBPL(const CollisionRobotSBPL& other) :
    CollisionRobot(other)
{
    ROS_INFO_NAMED(CRP_LOGGER, "CollisionRobotSBPL(other)");
    m_scm_config = other.m_scm_config;
    m_jcgm_map = other.m_jcgm_map;
    m_rcm = other.m_rcm;
    m_updater = other.m_updater;
    m_collision_pub = other.m_collision_pub;
}

CollisionRobotSBPL::~CollisionRobotSBPL()
{
    ROS_INFO_NAMED(CRP_LOGGER, "~CollisionRobotSBPL");
}

const sbpl::collision::RobotCollisionModelConstPtr&
CollisionRobotSBPL::robotCollisionModel() const
{
    return m_rcm;
}

void CollisionRobotSBPL::checkOtherCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& robot_state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state) const
{
    // TODO: implement
    setVacuousCollision(res);
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
    setVacuousCollision(res);
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
    setVacuousCollision(res);
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
    setVacuousCollision(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    setVacuousCollision(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    const_cast<CollisionRobotSBPL*>(this)->checkSelfCollisionMutable(
            req, res, state, acm);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    // TODO: implement
    setVacuousCollision(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    setVacuousCollision(res);
}

double CollisionRobotSBPL::distanceOther(
    const robot_state::RobotState& state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state) const
{
    // TODO: implement
    return -1.0;
}

double CollisionRobotSBPL::distanceOther(
    const robot_state::RobotState& state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    return -1.0;
}

double CollisionRobotSBPL::distanceSelf(
    const robot_state::RobotState& state) const
{
    // TODO: implement
    return -1.0;
}

double CollisionRobotSBPL::distanceSelf(
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    return -1.0;
}

void CollisionRobotSBPL::updatedPaddingOrScaling(
    const std::vector<std::string>& links)
{
    CollisionRobot::updatedPaddingOrScaling(links);
}

void CollisionRobotSBPL::setVacuousCollision(CollisionResult& res) const
{
    res.collision = true;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 0.0;
}

void CollisionRobotSBPL::checkSelfCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    using sbpl::collision::AttachedBodiesCollisionModel;
    using sbpl::collision::AttachedBodiesCollisionState;
    using sbpl::collision::RobotCollisionState;
    using sbpl::collision::SelfCollisionModel;

    if (state.getRobotModel()->getName() != m_rcm->name()) {
        ROS_ERROR_NAMED(CRP_LOGGER, "Collision Robot Model does not match Robot Model");
        setVacuousCollision(res);
        return;
    }

    // lookup the name of the corresponding collision group
    auto jgcgit = m_jcgm_map.find(req.group_name);
    const std::string& collision_group_name =
            jgcgit == m_jcgm_map.end() ? req.group_name : jgcgit->second;

    if (!m_rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(CRP_LOGGER, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    if (!m_grid) {
        ROS_DEBUG_NAMED(CRP_LOGGER, "Initialize self collision model");

        // lazily initialize self collision model
        m_grid = createGridFor(m_scm_config);
        m_grid->setReferenceFrame(m_rcm->modelFrame());

        auto bbma = m_grid->getOccupiedVoxelsVisualization();
        for (auto& m : bbma.markers) {
            m.ns = "self_collision_model_bounds";
        }
        m_collision_pub.publish(bbma);

        m_ab_model = std::make_shared<AttachedBodiesCollisionModel>(
                m_rcm.get());
        m_ab_state = std::make_shared<AttachedBodiesCollisionState>(
                m_ab_model.get(), m_updater.collisionState().get());
        m_scm = std::make_shared<SelfCollisionModel>(
                m_grid.get(), m_rcm.get(), m_ab_model.get());
    }

    updateAttachedBodies(state);

    int gidx = m_rcm->groupIndex(collision_group_name);

    m_updater.updateInternal(state);

    double dist;
    bool valid = m_scm->checkCollision(
            *m_updater.collisionState(),
            *m_ab_state,
            AllowedCollisionMatrixInterface(acm),
            gidx,
            dist);

    const bool verbose = req.verbose;
    if (verbose) {
        ROS_DEBUG_STREAM_NAMED(CRP_LOGGER, "valid: " << std::boolalpha << valid << ", dist: " << dist);
    }

    const bool visualize = req.verbose;
    if (visualize) {
        auto ma = getCollisionRobotVisualization(*m_updater.collisionState(), gidx);
        if (!valid) {
            for (auto& m : ma.markers) {
                m.color.r = 1.0;
                m.color.g = m.color.b = 0.0;
            }
        }
        m_collision_pub.publish(m_grid->getOccupiedVoxelsVisualization());
        m_collision_pub.publish(ma);
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

void CollisionRobotSBPL::updateAttachedBodies(
    const moveit::core::RobotState& state)
{
    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    state.getAttachedBodies(attached_bodies);

    // add bodies not in the attached body model
    for (const moveit::core::AttachedBody* ab : attached_bodies) {
        bool abm_updated = false;
        if (!m_ab_model->hasAttachedBody(ab->getName())) {
            abm_updated = true;
            ROS_INFO("Attach body '%s' from Robot Collision Model", ab->getName().c_str());
            m_ab_model->attachBody(ab->getName(), ab->getShapes(), ab->getFixedTransforms(), ab->getAttachedLinkName());
        }
    }

    // remove bodies not in the list of attached bodies
    if (m_ab_model->attachedBodyCount() > attached_bodies.size()) {
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
                ROS_INFO("Detach body '%s' from Robot Collision Model", ab_name.c_str());
                m_ab_model->detachBody(ab_name);
            }
        }
    }
}

double CollisionRobotSBPL::getSelfCollisionPropagationDistance() const
{
    // TODO: include the attached object models when computing the max expansion
    // distance

    // resolve the maximum expansion distance. this should be at least the
    // radius of the largest leaf sphere in the collision model but may be
    // overridden by the user to a larger value
    double cfg_max_distance_m = m_scm_config.max_distance_m;
    if (cfg_max_distance_m > 0.0) {
        // allow the user to set the maximum expansion distance. a value between
        // the max leaf sphere radius and the max sphere radius abandons some
        // efficiency gained by hierarchical checking in exchange for fewer
        // distance propagations. a value larger than the max sphere radius
        // provides additional cost information about how far the robot is from
        // environment obstacles.
        const double required_radius = m_rcm->maxLeafSphereRadius() + m_scm_config.res_m;
        if (cfg_max_distance_m < required_radius) {
            ROS_WARN_NAMED(CRP_LOGGER, "configured max distance set to %0.3f. overriding to required radius %0.3f", cfg_max_distance_m, required_radius);
        }
        return std::max(required_radius, cfg_max_distance_m);
    } else {
        // default to the value of the largest sphere (leaf and internal nodes)
        return m_rcm->maxSphereRadius() + m_scm_config.res_m;
    }
}

sbpl::OccupancyGridPtr CollisionRobotSBPL::createGridFor(
    const CollisionGridConfig& config) const
{
    ROS_DEBUG_NAMED(CRP_LOGGER, "  Creating Distance Field");
    ROS_DEBUG_NAMED(CRP_LOGGER, "    size: (%0.3f, %0.3f, %0.3f)", config.size_x, config.size_y, config.size_z);
    ROS_DEBUG_NAMED(CRP_LOGGER, "    origin: (%0.3f, %0.3f, %0.3f)", config.origin_x, config.origin_y, config.origin_z);
    ROS_DEBUG_NAMED(CRP_LOGGER, "    resolution: %0.3f", config.res_m);
    ROS_DEBUG_NAMED(CRP_LOGGER, "    max_distance: %0.3f", config.max_distance_m);

    // TODO: this can be substantially smaller since it only has to encompass
    // the range of motion of the robot
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

visualization_msgs::MarkerArray
CollisionRobotSBPL::getCollisionRobotVisualization(
    sbpl::collision::RobotCollisionState& rcs,
    int gidx) const
{
    auto ma = GetCollisionMarkers(rcs, gidx);
    for (auto& m : ma.markers) {
        m.ns = "self_collision";
        m.header.frame_id = m_rcm->modelFrame();
    }
    return ma;
}

} // namespace collision_detection
