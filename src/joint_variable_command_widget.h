#ifndef sbpl_interface_joint_variable_command_widget_h
#define sbpl_interface_joint_variable_command_widget_h

// standard includes
#include <map>
#include <vector>

// system includes
#include <QtGui>
#include <moveit/robot_model/robot_model.h>

namespace sbpl_interface {

class MoveGroupCommandModel;

class JointVariableCommandWidget : public QScrollArea
{
    Q_OBJECT

public:

    typedef QScrollArea Base;

    JointVariableCommandWidget(
        MoveGroupCommandModel* model,
        QWidget* parent = 0);
    ~JointVariableCommandWidget();

    const std::vector<QDoubleSpinBox*> spinboxes() const {
        return m_vind_to_spinbox;
    }

    int spinboxToVariableIndex(QDoubleSpinBox* spinbox) const;

    QDoubleSpinBox* variableIndexToSpinBox(int vind) const {
        return m_vind_to_spinbox[vind];
    }

    void displayJointGroupCommands(const std::string& joint_group_name);

private:

    MoveGroupCommandModel* m_model;

    // mapping from each qdoublespinbox to the index of the joint variable it
    // controls
    std::map<QDoubleSpinBox*, int> m_spinbox_to_vind;
    std::vector<QDoubleSpinBox*> m_vind_to_spinbox;
    std::vector<QLabel*> m_vind_to_label;

    QDoubleSpinBox* setupSpinBoxFor(
        const std::string& var_name,
        const moveit::core::VariableBounds& var_bounds,
        const moveit::core::JointModel& joint_model);
};

} // namespace sbpl_interface

#endif
