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
#include <sbpl_geometry_utils/interpolation.h>

namespace sbpl_interface {

MoveItCollisionChecker::MoveItCollisionChecker() :
    Base(),
    m_scene(),
    m_min_limits(),
    m_max_limits(),
    m_inc(),
    m_continuous()
{
}

MoveItCollisionChecker::~MoveItCollisionChecker()
{
}

bool MoveItCollisionChecker::init(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    const planning_scene::PlanningSceneConstPtr& scene)
{
    if (!scene) {
        return false;
    }

    m_scene = scene;
    return true;
}

bool MoveItCollisionChecker::isStateValid(
    const std::vector<double>& angles,
    bool verbose,
    bool visualize,
    double& dist)
{
    return false;
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
    if (!interpolatePath(angles0, angles1, m_inc, path)) {
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
            start, end, m_min_limits, m_max_limits, m_inc, m_continuous, path);
}

visualization_msgs::MarkerArray 
MoveItCollisionChecker::getCollisionModelVisualization(
    const std::vector<double>& angles)
{
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray 
MoveItCollisionChecker::getVisualization(const std::string& type)
{
    return visualization_msgs::MarkerArray();
}

} // namespace sbpl_interface
