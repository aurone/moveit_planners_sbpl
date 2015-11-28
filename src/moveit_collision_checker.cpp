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
#include <ros/console.h>
#include <sbpl_geometry_utils/interpolation.h>

#include <moveit_planners_sbpl/moveit_robot_model.h>

namespace sbpl_interface {

MoveItCollisionChecker::MoveItCollisionChecker() :
    Base(),
    m_robot_model(nullptr),
    m_scene(),
    m_ref_state()
{
}

MoveItCollisionChecker::~MoveItCollisionChecker()
{
}

bool MoveItCollisionChecker::init(
    const MoveItRobotModel* robot_model,
    const moveit::core::RobotState& ref_state,
    const planning_scene::PlanningSceneConstPtr& scene)
{
    ROS_INFO("Initializing MoveIt! Collision Checker");

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

    // TODO: this isn't necessary for the sbpl collision checker...do we need
    // this for FCL to function properly?
//    if (m_ref_state->dirtyLinkTransforms()) {
//        m_ref_state->updateLinkTransforms();
//    }

    // TODO: need to propagate path_constraints and trajectory_constraints down
    // to this level from the planning context. Once those are propagated, this
    // call will need to be paired with an additional call to isStateConstrained

    Eigen::MatrixXd J;
    J = m_ref_state->getJacobian(
            m_robot_model->planningJointGroup(), Eigen::Vector3d(0.17, 0.0, 0.0));

    double weight_lbs = 10.0;
    double kg_per_lb = 1.0 / 2.2;
    double gravity_z = -9.8;

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d f;
    f(0) = 0.0;
    f(1) = 0.0;
    f(2) = gravity_z * weight_lbs * kg_per_lb;
    f(3) = 0.0;
    f(4) = 0.0;
    f(5) = 0.0;

    Eigen::VectorXd t = J.transpose() * f;

    if (!m_scene->isStateColliding(
            *m_ref_state, m_robot_model->planningGroupName(), verbose))
    {
        return true;
    }
    else {
        dist = 0.0;
        return false;
    }
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
    if (!interpolatePath(
            angles0, angles1, m_robot_model->variableIncrements(), path))
    {
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
    const std::vector<double>& end,
    const std::vector<double>& inc,
    std::vector<std::vector<double>>& path)
{
    return sbpl::interp::InterpolatePath(
            start,
            end,
            m_robot_model->variableMinLimits(),
            m_robot_model->variableMaxLimits(),
            inc,
            m_robot_model->variableContinuous(),
            path);
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
    robot_state.getRobotMarkers(marker_arr, m_robot_model->moveitRobotModel()->getLinkModelNames(), color, "", ros::Duration(0));
    return marker_arr;
}

visualization_msgs::MarkerArray
MoveItCollisionChecker::getVisualization(const std::string& type)
{
    return visualization_msgs::MarkerArray();
}

} // namespace sbpl_interface
