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
    m_robot_state->update();
    Q_EMIT robotStateChanged();

    return true;
}

// TODO: CHECK FOR STATE CHANGES AND ONLY EMIT ROBOTSTATECHANGED IF IT
// ACTUALLY DID

void RobotCommandModel::setVariablePositions(const double* position)
{
    ROS_DEBUG_NAMED(LOG, "Set variable positions from array");
    m_robot_state->setVariablePositions(position);
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

void RobotCommandModel::setVariablePositions(const std::vector<double>& position)
{
    ROS_DEBUG_NAMED(LOG, "Set variable positions from vector");
    m_robot_state->setVariablePositions(position);
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

void RobotCommandModel::setVariablePositions(
    const std::map<std::string, double>& variable_map)
{
    ROS_DEBUG_NAMED(LOG, "Set variable positions from map");
    m_robot_state->setVariablePositions(variable_map);
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

void RobotCommandModel::setVariablePositions(
    const std::map<std::string, double>& variable_map,
    std::vector<std::string>& missing_variables)
{
    ROS_DEBUG_NAMED(LOG, "Set variable positions from map and report missing");
    m_robot_state->setVariablePositions(variable_map, missing_variables);
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

void RobotCommandModel::setVariablePositions(
    const std::vector<std::string>& variable_names,
    const std::vector<double>& variable_position)
{
    ROS_DEBUG_NAMED(LOG, "Set variable positions from name/position pairs");
    m_robot_state->setVariablePositions(variable_names, variable_position);
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

void RobotCommandModel::setVariablePosition(
    const std::string& variable,
    double value)
{
    ROS_DEBUG_NAMED(LOG, "Set position of variable '%s' to %f", variable.c_str(), value);
    m_robot_state->setVariablePosition(variable, value);
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

void RobotCommandModel::setVariablePosition(int index, double value)
{
    ROS_DEBUG_NAMED(LOG, "Set position of variable %d to %f", index, value);
    m_robot_state->setVariablePosition(index, value);
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

bool RobotCommandModel::setFromIK(
    const moveit::core::JointModelGroup* group,
    const Eigen::Affine3d& pose,
    unsigned int attempts,
    double timeout,
    const moveit::core::GroupStateValidityCallbackFn& constraint,
    const kinematics::KinematicsQueryOptions& options)
{
    ROS_DEBUG_NAMED(LOG, "Set positions of joint group '%s' via IK", group->getName().c_str());
    bool res = m_robot_state->setFromIK(group, pose, attempts, timeout, constraint, options);
    if (res) {
        m_robot_state->update();
        Q_EMIT robotStateChanged();
    }
    return res;
}

} // namespace sbpl_interface
