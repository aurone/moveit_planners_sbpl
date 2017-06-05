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

/// \author Andrew Dornbush

#include "moveit_collision_checker.h"

// standard includes
#include <limits>

// system includes
#include <leatherman/print.h>
#include <leatherman/viz.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <smpl/angles.h>

#include <moveit_planners_sbpl/moveit_robot_model.h>

namespace sbpl_interface {

MoveItCollisionChecker::MoveItCollisionChecker() :
    Base(),
    m_robot_model(nullptr),
    m_scene(),
    m_ref_state()
{
    ros::NodeHandle nh;
    m_vpub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);
}

MoveItCollisionChecker::~MoveItCollisionChecker()
{
}

bool MoveItCollisionChecker::init(
    MoveItRobotModel* robot_model,
    const moveit::core::RobotState& ref_state,
    const planning_scene::PlanningSceneConstPtr& scene)
{
    ROS_DEBUG("Initializing MoveIt! Collision Checker");

    if (!robot_model->initialized()) {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "MoveIt Robot Model must be initialized");
        return false;
    }

    if (!scene) {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "Planning Scene is null");
        return false;
    }

    if (robot_model->moveitRobotModel()->getName() !=
        scene->getRobotModel()->getName())
    {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "model is not the same between SBPL Robot Model and Planning Scene");
        return false;
    }

    if (robot_model->moveitRobotModel()->getName() !=
        ref_state.getRobotModel()->getName())
    {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "model is not the same between SBPL Robot Model and reference state");
        return false;
    }

    m_robot_model = robot_model;

    m_var_incs.reserve(m_robot_model->getPlanningJoints().size());
    for (const std::string& joint_name : m_robot_model->getPlanningJoints()) {
        m_var_incs.push_back(sbpl::angles::to_radians(2.0));
    }
    ROS_INFO("Increments: %s", to_string(m_var_incs).c_str());

    m_ref_state.reset(new moveit::core::RobotState(scene->getRobotModel()));
    *m_ref_state = ref_state;

    m_scene = scene;

    m_zero_state.resize(m_robot_model->activeVariableCount(), 0.0);

    ros::NodeHandle ph("~");
    ph.param("enable_ccd", m_enabled_ccd, false);
    ROS_INFO("enable_ccd: %s", m_enabled_ccd ? "true" : "false");

    return true;
}

bool MoveItCollisionChecker::initialized() const
{
    return (bool)m_robot_model;
}

sbpl::motion::Extension* MoveItCollisionChecker::getExtension(size_t class_code)
{
    if (class_code == sbpl::motion::GetClassCode<sbpl::motion::CollisionChecker>()) {
        return this;
    }
    return nullptr;
}

bool MoveItCollisionChecker::isStateValid(
    const sbpl::motion::RobotState& state,
    bool verbose,
    bool visualize,
    double& dist)
{
    if (!initialized()) {
        ROS_ERROR("MoveItCollisionChecker is not initialized");
        return false;
    }

    // fill in variable values
    for (size_t vind = 0; vind < state.size(); ++vind) {
        m_ref_state->setVariablePosition(
                m_robot_model->activeVariableIndices()[vind], state[vind]);
    }

    // TODO: need to propagate path_constraints and trajectory_constraints down
    // to this level from the planning context. Once those are propagated, this
    // call will need to be paired with an additional call to isStateConstrained

    // NOTE: since m_ref_state is not const, this call to isStateColliding will
    // go ahead and update the link transforms underneath before checking for
    // collisions. Source:
    //
    // http://docs.ros.org/indigo/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html
    //
    if (m_scene->isStateColliding(
            *m_ref_state, m_robot_model->planningGroupName(), verbose))
    {
        dist = 0.0;
        if (visualize) {
            auto ma = getCollisionModelVisualization(state);
            for (auto& marker : ma.markers) {
                marker.color.r = 0.8;
                marker.ns = "collision";
            }
            m_vpub.publish(ma);
        }
        return false;
    }

    return true;
}

bool MoveItCollisionChecker::isStateToStateValid(
    const sbpl::motion::RobotState& start,
    const sbpl::motion::RobotState& finish,
    int& path_length,
    int& num_checks,
    double& dist)
{
    if (m_enabled_ccd) {
        collision_detection::CollisionRequest req;
        req.verbose = false;
        req.group_name = m_robot_model->planningGroupName();
        collision_detection::CollisionResult res;

        auto cw = m_scene->getCollisionWorld();
        moveit::core::RobotState state1(*m_ref_state);
        moveit::core::RobotState state2(*m_ref_state);
        for (size_t vind = 0; vind < start.size(); ++vind) {
            state1.setVariablePosition(
                    m_robot_model->activeVariableIndices()[vind], start[vind]);
            state2.setVariablePosition(
                    m_robot_model->activeVariableIndices()[vind], finish[vind]);
        }
        cw->checkRobotCollision(
                req, res, *m_scene->getCollisionRobot(), state1, state2, m_scene->getAllowedCollisionMatrix());
        if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts)) {
            auto cr = m_scene->getCollisionRobotUnpadded();
            cr->checkSelfCollision(
                    req, res, state1, state2, m_scene->getAllowedCollisionMatrix());
        }

        return !res.collision;
    } else {
        path_length = 0; num_checks = 0; dist = std::numeric_limits<double>::max();
        int waypoint_count = interpolatePathFast(start, finish, m_waypoint_path);
        if (waypoint_count < 0) {
            return false;
        }

        for (int widx = 0; widx < waypoint_count; ++widx) {
            const sbpl::motion::RobotState& p = m_waypoint_path[widx];
            double d;
            bool res = isStateValid(p, false, false, d);
            if (d < dist) {
                dist = d;
            }
            ++num_checks;
            if (!res) {
                return false;
            }
        }

        return true;
    }
}

bool MoveItCollisionChecker::interpolatePath(
    const sbpl::motion::RobotState& start,
    const sbpl::motion::RobotState& finish,
    std::vector<sbpl::motion::RobotState>& opath)
{
    opath.clear();
    return interpolatePathFast(start, finish, opath) >= 0;
}

int MoveItCollisionChecker::interpolatePathFast(
    const sbpl::motion::RobotState& start,
    const sbpl::motion::RobotState& finish,
    std::vector<sbpl::motion::RobotState>& opath)
{
    assert(start.size() == m_robot_model->activeVariableCount() &&
            finish.size() == m_robot_model->activeVariableCount());

    // check joint limits on the start and finish points
    if (!(m_robot_model->checkJointLimits(start) &&
            m_robot_model->checkJointLimits(finish)))
    {
        ROS_ERROR("Joint limits violated");
        return -1;
    }

    // compute distance traveled by each joint
    m_diffs.resize(m_robot_model->activeVariableCount(), 0.0);
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
        if (m_robot_model->variableContinuous()[vidx]) {
            m_diffs[vidx] = sbpl::angles::shortest_angle_diff(finish[vidx], start[vidx]);
        }
        else {
            m_diffs[vidx] = finish[vidx] - start[vidx];
        }
    }

    // compute the number of intermediate waypoints including start and end
    int waypoint_count = 0;
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); vidx++) {
        int angle_waypoints = (int)(std::fabs(m_diffs[vidx]) / m_var_incs[vidx]) + 1;
        waypoint_count = std::max(waypoint_count, angle_waypoints);
    }
    waypoint_count = std::max(waypoint_count, 2);

    // fill intermediate waypoints
    const int prev_size = (int)opath.size();
    if (waypoint_count > prev_size) {
        opath.resize(waypoint_count, m_zero_state);
    }
    for (size_t widx = 0; widx < waypoint_count; ++widx) {
        for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
            double alpha = (double)widx / (double)(waypoint_count - 1);
            double pos = start[vidx] + alpha * m_diffs[vidx];
            opath[widx][vidx] = pos;
        }
    }

    // normalize output continuous variables
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
        if (m_robot_model->variableContinuous()[vidx]) {
            for (size_t widx = 0; widx < waypoint_count; ++widx) {
                opath[widx][vidx] = sbpl::angles::normalize_angle(opath[widx][vidx]);
            }
        }
    }

    return waypoint_count;
}

visualization_msgs::MarkerArray
MoveItCollisionChecker::getCollisionModelVisualization(
    const sbpl::motion::RobotState& state)
{
    moveit::core::RobotState robot_state(*m_ref_state);

    for (size_t vind = 0; vind < state.size(); ++vind) {
        int avind = m_robot_model->activeVariableIndices()[vind];
        robot_state.setVariablePosition(avind, state[vind]);
    }

    visualization_msgs::MarkerArray marker_arr;
    std_msgs::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.8;
    color.a = 0.8;
    robot_state.getRobotMarkers(
            marker_arr,
            m_robot_model->moveitRobotModel()->getLinkModelNames(),
            color,
            "",
            ros::Duration(0),
            true);

    const moveit::core::LinkModel* tip_link = m_robot_model->planningTipLink();
    if (tip_link) {
        const Eigen::Affine3d& T_model_tip =
                robot_state.getGlobalLinkTransform(tip_link->getName());
        auto frame_markers = viz::getFrameMarkerArray(
                T_model_tip, m_robot_model->moveitRobotModel()->getModelFrame(), "", marker_arr.markers.size());
        marker_arr.markers.insert(marker_arr.markers.end(), frame_markers.markers.begin(), frame_markers.markers.end());
    }

    return marker_arr;
}

visualization_msgs::MarkerArray
MoveItCollisionChecker::getVisualization(const std::string& type)
{
    return visualization_msgs::MarkerArray();
}

} // namespace sbpl_interface
