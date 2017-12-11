#ifndef MOVEIT_PLANNERS_SBPL_TELEOP_COMMAND_H
#define MOVEIT_PLANNERS_SBPL_TELEOP_COMMAND_H

#include <QObject>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace sbpl_interface {

class RobotCommandModel;

class TeleopCommand : QObject
{
    Q_OBJECT

public:

    TeleopCommand(RobotCommandModel* model);

    auto getActiveJointGroup() const -> const std::string& {
        return m_active_group_name;
    }

public Q_SLOTS:

    void setActiveJointGroup(const std::string& group_name);

Q_SIGNALS:

    void updateActiveJointGroup(const std::string& group_name);

private:

    RobotCommandModel* m_model = nullptr;
    std::string m_active_group_name;

    ros::NodeHandle m_nh;
    ros::Subscriber m_joy_sub;

    sensor_msgs::Joy::ConstPtr m_prev_joy;
    int m_remote_curr_var = -1;

private Q_SLOTS:

    void updateRobotModel();
    void updateRobotState();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    void cycleActiveJoint(bool forward);
    void cycleActiveGroup(bool forward);
    void moveActiveJoint(double dp);
};

} // namespace sbpl_interface

#endif
