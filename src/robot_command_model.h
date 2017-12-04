#ifndef MOVEIT_PLANNERS_SBPL_ROBOT_COMMAND_MODEL_H
#define MOVEIT_PLANNERS_SBPL_ROBOT_COMMAND_MODEL_H

#include <string>

#include <QObject>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace sbpl_interface {

/// The interface used by JointVariableCommandWidget to control the state of the
/// robot command using spinbox controls.
class RobotCommandModel : public QObject
{
    Q_OBJECT

public:

    virtual ~RobotCommandModel();

    virtual const moveit::core::RobotModel* getRobotModel() const = 0;
    virtual const moveit::core::RobotState* getRobotState() const = 0;
    virtual const std::string& getPlanningJointGroupName() const = 0;
    virtual void setPlanningJointGroup(const std::string& group_name) = 0;
    virtual void setJointVariable(int index, double value) = 0;

Q_SIGNALS:

    void robotLoaded();
};

} // namespace sbpl_interface

#endif
