#ifndef MOVEIT_PLANNERS_SBPL_ROBOT_COMMAND_MODEL_H
#define MOVEIT_PLANNERS_SBPL_ROBOT_COMMAND_MODEL_H

#include <string>

#include <QObject>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace sbpl_interface {

// A wrapper around moveit::core::RobotState to provide a QObject-enabled
// RobotState that signals state changes
class RobotCommandModel : public QObject
{
    Q_OBJECT

public:

    ~RobotCommandModel();

    // Load a new RobotModel into the RobotCommandModel. Emits the robotLoaded()
    // signal, sets the managed RobotState to its default values, and emits a
    // robotStateChanged() signal.
    bool load(const moveit::core::RobotModelConstPtr& robot);

    /// Returns the loaded RobotModel, null if no RobotModel has been loaded.
    auto getRobotModel() const -> const moveit::core::RobotModelConstPtr& {
        return m_robot_model;
    }

    /// Return a pointer to the managed RobotState, null if no RobotModel has
    /// been loaded.
    auto getRobotState() const -> const moveit::core::RobotState* {
        return m_robot_state.get();
    }

    void setVariablePositions(const double* position);

    void setVariablePositions(const std::vector<double>& position);

    void setVariablePositions(
        const std::map<std::string, double>& variable_map);

    void setVariablePositions(
        const std::map<std::string, double>& variable_map,
        std::vector<std::string>& missing_variables);

    void setVariablePositions(
        const std::vector<std::string>& variable_names,
        const std::vector<double>& variable_position);

    void setVariablePosition(const std::string& variable, double value);

    void setVariablePosition(int index, double value);

    bool setFromIK(
        const moveit::core::JointModelGroup* group,
        const Eigen::Affine3d& pose,
        unsigned int attempts = 0,
        double timeout = 0.0,
        const moveit::core::GroupStateValidityCallbackFn& constraint = moveit::core::GroupStateValidityCallbackFn(),
        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

    void setJointGroupPositions(
        const moveit::core::JointModelGroup* group,
        const std::vector<double>& positions);

    bool setToDefaultValues(
        const moveit::core::JointModelGroup* group,
        const std::string& name = "");

    void setJointPositions(
        const moveit::core::JointModel* joint,
        const Eigen::Affine3d& joint_transform);

Q_SIGNALS:

    void robotLoaded();
    void robotStateChanged();

private:

    moveit::core::RobotModelConstPtr m_robot_model = nullptr;
    std::unique_ptr<moveit::core::RobotState> m_robot_state;

    void updateAndNotify();
};

} // namespace sbpl_interface

#endif
