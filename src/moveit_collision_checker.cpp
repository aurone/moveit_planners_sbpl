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
#include <ros/console.h>
#include <ros/ros.h>
#include <sbpl_geometry_utils/utils.h>

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

    m_ref_state.reset(new moveit::core::RobotState(scene->getRobotModel()));
    *m_ref_state = ref_state;

    m_scene = scene;

    // populate min_limits, max_limits, inc, and continuous
    const moveit::core::JointModelGroup* joint_group =
            robot_model->planningJointGroup();

    const std::vector<std::string>& planning_var_names =
            robot_model->planningVariableNames();
    for (size_t vind = 0; vind < planning_var_names.size(); ++vind) {
        const std::string& var_name = planning_var_names[vind];
    }

    return true;
}

bool MoveItCollisionChecker::initialized() const
{
    return (bool)m_robot_model;
}

bool MoveItCollisionChecker::isStateValid(
    const std::vector<double>& angles,
    bool verbose,
    bool visualize,
    double& dist)
{
    if (!initialized()) {
        ROS_ERROR("MoveItCollisionChecker is not initialized");
        return false;
    }

    // fill in variable values
    for (size_t vind = 0; vind < angles.size(); ++vind) {
        m_ref_state->setVariablePosition(
                m_robot_model->activeVariableIndices()[vind], angles[vind]);
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
            visualization_msgs::MarkerArray marr =
                    getCollisionModelVisualization(angles);
            for (auto& marker : marr.markers) {
                marker.color.r = 0.8;
                marker.ns = "collision";
            }
            m_vpub.publish(marr);
        }
        return false;
    }

    return true;
}

bool MoveItCollisionChecker::isStateToStateValid(
    const std::vector<double>& angles0,
    const std::vector<double>& angles1,
    int& path_length,
    int& num_checks,
    double& dist)
{
    path_length = 0;
    num_checks = 0;
    dist = std::numeric_limits<double>::max();

    std::vector<std::vector<double>> path;
    if (!interpolatePath(angles0, angles1, path)) {
        return false;
    }

    for (const std::vector<double>& p : path) {
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

bool MoveItCollisionChecker::interpolatePath(
    const std::vector<double>& start,
    const std::vector<double>& finish,
    std::vector<std::vector<double>>& opath)
{
    assert(start.size() == m_robot_model->activeVariableCount() &&
            finish.size() == m_robot_model->activeVariableCount());

    // check joint limits on the start and finish points
    if (!(m_robot_model->checkJointLimits(start) &&
            m_robot_model->checkJointLimits(finish)))
    {
        ROS_ERROR("Joint limits violated");
        return false;
    }

    // compute distance traveled by each joint
    std::vector<double> diffs(m_robot_model->activeVariableCount(), 0.0);
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
        if (m_robot_model->variableContinuous()[vidx]) {
            diffs[vidx] = sbpl::angles::ShortestAngleDiff(finish[vidx], start[vidx]);
        }
        else {
            diffs[vidx] = finish[vidx] - start[vidx];
        }
    }

    // compute the number of intermediate waypoints including start and end
    int waypoint_count = 0;
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); vidx++) {
        int angle_waypoints = (int)(std::fabs(diffs[vidx]) / m_robot_model->variableIncrements()[vidx]) + 1;
        waypoint_count = std::max(waypoint_count, angle_waypoints);
    }
    waypoint_count = std::max(waypoint_count, 2);

    // fill intermediate waypoints
    std::vector<std::vector<double>> path(
            waypoint_count,
            std::vector<double>(m_robot_model->activeVariableCount(), 0.0));
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
        for (size_t widx = 0; widx < waypoint_count; ++widx) {
            double alpha = (double)widx / (double)(waypoint_count - 1);
            double pos = start[vidx] + alpha * diffs[vidx];
            path[widx][vidx] = pos;
        }
    }

    // normalize output angles
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
        if (m_robot_model->variableContinuous()[vidx]) {
            for (size_t widx = 0; widx < waypoint_count; ++widx) {
                path[widx][vidx] = sbpl::angles::NormalizeAngle(path[widx][vidx]);
            }
        }
    }

    opath = std::move(path);
    return true;
}

visualization_msgs::MarkerArray
MoveItCollisionChecker::getCollisionModelVisualization(
    const std::vector<double>& angles)
{
    moveit::core::RobotState robot_state(*m_ref_state);

    for (size_t vind = 0; vind < angles.size(); ++vind) {
        int avind = m_robot_model->activeVariableIndices()[vind];
        robot_state.setVariablePosition(avind, angles[vind]);
    }

    // TODO: get all links that are descendants of this joint group
//    const moveit::core::JointModelGroup* joint_group =
//            m_robot_model->planningJointGroup();
//    const std::vector<std::string>& link_names =
//            joint_group->getLinkModelNames();

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
            ros::Duration(0));
    return marker_arr;
}

visualization_msgs::MarkerArray
MoveItCollisionChecker::getVisualization(const std::string& type)
{
    return visualization_msgs::MarkerArray();
}

} // namespace sbpl_interface
