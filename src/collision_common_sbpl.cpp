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

#include <moveit_planners_sbpl/collision_common_sbpl.h>

// standard includes
#include <sstream>
#include <stdexcept>

// system includes
#include <leatherman/print.h>

namespace collision_detection {

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

    // check for required type and members
    if (cm_config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !cm_config.hasMember("size_x") ||
        !cm_config.hasMember("size_y") ||
        !cm_config.hasMember("size_z") ||
        !cm_config.hasMember("origin_x") ||
        !cm_config.hasMember("origin_y") ||
        !cm_config.hasMember("origin_z") ||
        !cm_config.hasMember("res_m") ||
        !cm_config.hasMember("max_distance_m"))
    {
        std::stringstream ss;
        ss << "'" << param_name << "' param is malformed";
        ROS_ERROR_STREAM(ss.str());
        ROS_ERROR_STREAM("has size_x member " << cm_config.hasMember("size_x"));
        ROS_ERROR_STREAM("has size_y member " << cm_config.hasMember("size_y"));
        ROS_ERROR_STREAM("has size_z member " << cm_config.hasMember("size_z"));
        ROS_ERROR_STREAM("has origin_x member " << cm_config.hasMember("origin_x"));
        ROS_ERROR_STREAM("has origin_y member " << cm_config.hasMember("origin_y"));
        ROS_ERROR_STREAM("has origin_z member " << cm_config.hasMember("origin_z"));
        ROS_ERROR_STREAM("has res_m member " << cm_config.hasMember("res_m"));
        throw std::runtime_error(ss.str());
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

bool ExtractRobotVariables(
    const moveit::core::RobotModel& robot_model,
    std::vector<std::string>& variable_names,
    std::vector<int>& variable_indices,
    bool& are_variables_contiguous,
    int& variables_offset)
{
    // Figure out how to extract the internal robot state from a complete robot
    // state. We're just going to send the entire internal robot state variables
    // to isStateValid calls since there isn't a notion of planning variables
    // here. The other robot state variables will be set via the world to model
    // transform in the collision space

    std::vector<std::string> robot_var_names;
    std::vector<int> robot_var_indices;
    if (!GetRobotVariableNames(robot_model, robot_var_names, robot_var_indices))
    {
        return false;
    }

    assert(robot_var_names.size() == robot_var_indices.size());
    // could also assert that the robot variable indices are sorted here

    bool contiguous = true;
    for (size_t i = 1; i < robot_var_indices.size(); ++i) {
        if (robot_var_indices[i] != robot_var_indices[i - 1] + 1) {
            contiguous = false;
            break;
        }
    }

    variable_names = robot_var_names;
    variable_indices = robot_var_indices;
    are_variables_contiguous = contiguous;
    variables_offset = 0;
    if (are_variables_contiguous && !robot_var_names.empty()) {
        variables_offset = std::distance(
                robot_model.getVariableNames().begin(),
                std::find(
                        robot_model.getVariableNames().begin(),
                        robot_model.getVariableNames().end(),
                        robot_var_names.front()));
    }

    ROS_DEBUG("Sorted Variable Names: %s", to_string(variable_names).c_str());
    ROS_DEBUG("Sorted Variable Indices: %s", to_string(variable_indices).c_str());
    ROS_DEBUG("Contiguous: %s", are_variables_contiguous ? "true" : "false");
    ROS_DEBUG("Variables Offset: %d", variables_offset);
    return true;
}

/// Traverse a robot model to extract all variable names corresponding to the
/// URDF model (disregarding any virtual joints). The returned variables are
/// sorted by the order they appear in the robot model.
///
/// \return true if the model has a valid root link; false otherwise
bool GetRobotVariableNames(
    const moveit::core::RobotModel& model,
    std::vector<std::string>& var_names,
    std::vector<int>& var_indices)
{
    // traverse the robot kinematic structure to gather all the joint variables
    const moveit::core::LinkModel* root_link = model.getRootLink();
    if (!root_link) {
        return false;
    }

    // get all descendant joint models
    std::vector<const moveit::core::JointModel*> robot_joint_models;
    for (auto joint_model : root_link->getChildJointModels()) {
        robot_joint_models.insert(
                robot_joint_models.end(),
                joint_model->getDescendantJointModels().begin(),
                joint_model->getDescendantJointModels().end());
    }
    robot_joint_models.insert(
            robot_joint_models.end(),
            root_link->getChildJointModels().begin(),
            root_link->getChildJointModels().end());

    // aggregate all variable names
    std::vector<std::string> variable_names;
    for (auto joint_model : robot_joint_models) {
        variable_names.insert(
                variable_names.end(),
                joint_model->getVariableNames().begin(),
                joint_model->getVariableNames().end());
    }

    // sort by their order in the robot model variable vector
    std::sort(variable_names.begin(), variable_names.end(),
            [&](const std::string& var_a, const std::string& var_b)
            {
                size_t va_idx = std::distance(
                        model.getVariableNames().begin(),
                        std::find(model.getVariableNames().begin(), model.getVariableNames().end(), var_a));
                size_t vb_idx = std::distance(
                        model.getVariableNames().begin(),
                        std::find(model.getVariableNames().begin(), model.getVariableNames().end(), var_b));
                return va_idx < vb_idx;
            });


    std::vector<int> variable_indices;
    for (const auto& var_name : variable_names) {
        variable_indices.push_back(std::distance(
                model.getVariableNames().begin(),
                std::find(model.getVariableNames().begin(), model.getVariableNames().end(), var_name)));
    }

    var_names = std::move(variable_names);
    var_indices = std::move(variable_indices);
    return true;
}

bool GetRobotCollisionModelJointIndices(
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

} // namespace collision_detection
