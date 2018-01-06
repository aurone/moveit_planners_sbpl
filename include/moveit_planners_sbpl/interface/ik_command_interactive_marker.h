#ifndef MOVEIT_PLANNERS_SBPL_IK_COMMAND_INTERACTIVE_MARKER_H
#define MOVEIT_PLANNERS_SBPL_IK_COMMAND_INTERACTIVE_MARKER_H

#include <string>
#include <vector>

#include <QtCore>
#ifndef Q_MOC_RUN
#include <interactive_markers/interactive_marker_server.h>
#endif

namespace sbpl_interface {

class RobotCommandModel;

class IKCommandInteractiveMarker : public QObject
{
    Q_OBJECT

public:

    IKCommandInteractiveMarker(RobotCommandModel* model);

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

    interactive_markers::InteractiveMarkerServer m_im_server;
    std::vector<std::string> m_int_marker_names;

    void reinitInteractiveMarkers();
    void updateInteractiveMarkers();

    void processInteractiveMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);

private Q_SLOTS:

    void updateRobotModel();
    void updateRobotState();
};

} // namespace sbpl_interface

#endif
