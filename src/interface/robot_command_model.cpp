#include <moveit_planners_sbpl/interface/robot_command_model.h>

namespace sbpl_interface {

static const char* LOG = "robot_command_model";

RobotCommandModel::~RobotCommandModel() { }

// Load a new RobotModel into the RobotCommandModel. Emits the robotLoaded()
// signal, sets the managed RobotState to its default values, and emits a
// robotStateChanged() signal.
bool RobotCommandModel::load(const moveit::core::RobotModelConstPtr& robot)
{
    if (!robot) {
        return false;
    }

    m_robot_model = robot;
    Q_EMIT robotLoaded();

    m_robot_state.reset(new moveit::core::RobotState(robot));
    m_robot_state->setToDefaultValues();
    updateAndNotify();

    return true;
}

// TODO: CHECK FOR STATE CHANGES AND ONLY EMIT ROBOTSTATECHANGED IF IT
// ACTUALLY DID

void RobotCommandModel::setVariablePositions(const double* position)
{
    const bool same = std::equal(
            position,
            position + m_robot_state->getVariableCount(),
            m_robot_state->getVariablePositions());
    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from array");
        m_robot_state->setVariablePositions(position);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePositions(const std::vector<double>& position)
{
    const bool same = std::equal(
            begin(position), end(position),
            m_robot_state->getVariablePositions());
    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from vector");
        m_robot_state->setVariablePositions(position);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePositions(
    const std::map<std::string, double>& variable_map)
{
    bool same = true;
    for (auto& e : variable_map) {
        if (m_robot_state->getVariablePosition(e.first) != e.second) {
            same = false;
            break;
        }
    }
    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from map");
        m_robot_state->setVariablePositions(variable_map);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePositions(
    const std::map<std::string, double>& variable_map,
    std::vector<std::string>& missing_variables)
{
    // TODO: detect differences
    ROS_DEBUG_NAMED(LOG, "Set variable positions from map and report missing");
    m_robot_state->setVariablePositions(variable_map, missing_variables);
    updateAndNotify();
}

void RobotCommandModel::setVariablePositions(
    const std::vector<std::string>& variable_names,
    const std::vector<double>& variable_position)
{
    assert(variable_names.size() == variable_position.size());
    bool same = true;
    for (size_t i = 0; i < variable_names.size(); ++i) {
        auto& name = variable_names[i];
        auto& position = variable_position[i];
        if (m_robot_state->getVariablePosition(name) != position) {
            same = false;
            break;
        }
    }
    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from name/position pairs");
        m_robot_state->setVariablePositions(variable_names, variable_position);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePosition(
    const std::string& variable,
    double value)
{
    if (m_robot_state->getVariablePosition(variable) != value) {
        ROS_DEBUG_NAMED(LOG, "Set position of variable '%s' to %f", variable.c_str(), value);
        m_robot_state->setVariablePosition(variable, value);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePosition(int index, double value)
{
    if (m_robot_state->getVariablePosition(index) != value) {
        ROS_DEBUG_NAMED(LOG, "Set position of variable %d to %f", index, value);
        m_robot_state->setVariablePosition(index, value);
        updateAndNotify();
    }
}

bool RobotCommandModel::setFromIK(
    const moveit::core::JointModelGroup* group,
    const Eigen::Affine3d& pose,
    unsigned int attempts,
    double timeout,
    const moveit::core::GroupStateValidityCallbackFn& constraint,
    const kinematics::KinematicsQueryOptions& options)
{
    // TODO: detect changes?
    bool res = m_robot_state->setFromIK(group, pose, attempts, timeout, constraint, options);
    if (res) {
        ROS_DEBUG_NAMED(LOG, "Set positions of joint group '%s' via IK", group->getName().c_str());
        updateAndNotify();
    }
    return res;
}

bool RobotCommandModel::setToDefaultValues(
    const moveit::core::JointModelGroup* group,
    const std::string& name)
{
    // TODO: detect changes?
    if (m_robot_state->setToDefaultValues(group, name)) {
        ROS_DEBUG_NAMED(LOG, "Set positions of joint group '%s' to default values '%s'", group->getName().c_str(), name.c_str());
        updateAndNotify();
        return true;
    }
    return false;
}

void RobotCommandModel::setJointPositions(
    const moveit::core::JointModel* joint,
    const Eigen::Affine3d& joint_transform)
{
    // TODO: detect changes?
    m_robot_state->setJointPositions(joint, joint_transform);
    updateAndNotify();
}

void RobotCommandModel::setJointGroupPositions(
    const moveit::core::JointModelGroup* group,
    const std::vector<double>& positions)
{
    // TODO: detect changes?
    ROS_DEBUG_NAMED(LOG, "Set positions of joint group '%s'", group->getName().c_str());
    m_robot_state->setJointGroupPositions(group, positions);
    updateAndNotify();
}

void RobotCommandModel::updateAndNotify()
{
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

} // namespace sbpl_interface
