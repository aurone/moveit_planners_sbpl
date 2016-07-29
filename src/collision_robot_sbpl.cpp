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

CollisionRobotSBPL::CollisionRobotSBPL(
    const robot_model::RobotModelConstPtr& model,
    double padding,
    double scale)
:
    CollisionRobot(model, padding, scale)
{
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

    // ok! store the robot collision model
    m_rcm = rcm;
}

CollisionRobotSBPL::CollisionRobotSBPL(const CollisionRobotSBPL& other) :
    CollisionRobot(other)
{
    m_rcm = other.m_rcm;
}

CollisionRobotSBPL::~CollisionRobotSBPL()
{
}

void CollisionRobotSBPL::checkOtherCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& robot_state,
    const CollisionRobot& other_robot,
    const robot_state::RobotState& other_state) const
{
    // TODO: implement
    clearAllCollisions(res);
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
    clearAllCollisions(res);
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
    clearAllCollisions(res);
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
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    // TODO: implement
    clearAllCollisions(res);
}

void CollisionRobotSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    clearAllCollisions(res);
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

void CollisionRobotSBPL::clearAllCollisions(CollisionResult& res) const
{
    res.collision = false;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 100.0;
}

} // namespace collision_detection
