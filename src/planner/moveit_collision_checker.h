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

#ifndef sbpl_interface_moveit_collision_checker_h
#define sbpl_interface_moveit_collision_checker_h

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <smpl/collision_checker.h>

namespace sbpl_interface {

class MoveItRobotModel;

class MoveItCollisionChecker : public smpl::CollisionChecker
{
public:

    typedef smpl::CollisionChecker Base;

    MoveItCollisionChecker();
    ~MoveItCollisionChecker();

    bool init(
        MoveItRobotModel* robot_model,
        const moveit::core::RobotState& ref_state,
        const planning_scene::PlanningSceneConstPtr& scene);

    bool initialized() const;

    /// \name Required Functions from Extension
    ///@{
    smpl::Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Functions from CollisionChecker
    ///@{
    bool isStateValid(
        const smpl::RobotState& angles,
        bool verbose) override;

    bool isStateToStateValid(
        const smpl::RobotState& angles0,
        const smpl::RobotState& angles1,
        bool verbose) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& opath) override;
    ///@}

    /// \name Reimplemented Functions from CollisionChecker
    ///@{
    auto getCollisionModelVisualization(const smpl::RobotState& angles)
        -> std::vector<smpl::visual::Marker> override;
    ///@}

private:

    MoveItRobotModel* m_robot_model;
    std::vector<double> m_var_incs;

    planning_scene::PlanningSceneConstPtr m_scene;

    moveit::core::RobotStatePtr m_ref_state;

    smpl::RobotState m_zero_state;
    std::vector<double> m_diffs;
    std::vector<smpl::RobotState> m_waypoint_path;

    bool m_enabled_ccd;

    auto checkContinuousCollision(
        const smpl::RobotState& start,
        const smpl::RobotState& finish)
        -> bool;

    auto checkInterpolatedPathCollision(
        const smpl::RobotState& start,
        const smpl::RobotState& finish)
        -> bool;

    void setRobotStateFromState(
        moveit::core::RobotState& robot_state,
        const smpl::RobotState& state) const;

    // interpolate the path between start and finish, storing intermediate
    // waypoints within opath. previous entries in opath are overwritten and
    // never cleared. the number of relevant waypoints is returned
    int interpolatePathFast(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& opath);
};

} // namespace sbpl_interface

#endif
