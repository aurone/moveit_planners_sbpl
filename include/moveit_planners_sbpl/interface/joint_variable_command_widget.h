#ifndef sbpl_interface_joint_variable_command_widget_h
#define sbpl_interface_joint_variable_command_widget_h

// standard includes
#include <map>
#include <vector>

// system includes
#include <QtGui>
#include <moveit/robot_model/robot_model.h>

namespace sbpl_interface {

class RobotCommandModel;

class JointVariableCommandWidget : public QWidget //QScrollArea
{
    Q_OBJECT

public:

    typedef QWidget Base;

    JointVariableCommandWidget(RobotCommandModel* model, QWidget* parent = 0);
    ~JointVariableCommandWidget();

    const std::string& activeJointGroup() const { return m_active_joint_group; }

public Q_SLOTS:

    void setActiveJointGroup(const std::string& group_name);

Q_SIGNALS:

    // emitted whenever the active joint group changes
    void updateActiveJointGroup(const std::string& group_name);

private:

    RobotCommandModel* m_model;

    std::string m_active_joint_group;

    QComboBox* m_joint_groups_combo_box;

    std::vector<QDoubleSpinBox*> m_spinboxes;
    std::vector<QLabel*> m_labels;

    // mapping from each qdoublespinbox to the indices of the joint variables it
    // controls...really this is a huge over-generalization to allow a set of
    // (r, p, y) spinboxes to control a set of (qw, qx, qy, qz) joint variables.
    // In those places, the indices are expected refer to joint variables in the
    // order (qw, qx, qy, qz).
    std::map<QDoubleSpinBox*, std::vector<int>> m_spinbox_to_vind;

    // mapping from variable index to the spinboxes that affects its value. As
    // above, this is to support RPY spinboxes. and in those cases, the
    // expected order of these spinboxes is (r, p, y)
    std::vector<std::vector<QDoubleSpinBox*>> m_vind_to_spinbox;

    // ...and their corresponding labels
    std::vector<std::vector<QLabel*>> m_vind_to_label;

    bool m_ignore_sync;

    void buildRobotControls();

    void displayJointGroupControls();

    QDoubleSpinBox* createRealVariableSpinBox(
        const moveit::core::VariableBounds& bounds);

    QDoubleSpinBox* createRollVariableSpinBox();
    QDoubleSpinBox* createPitchVariableSpinBox();
    QDoubleSpinBox* createYawVariableSpinBox();

    QDoubleSpinBox* createAngleVariableSpinBox();

    QDoubleSpinBox* createRevoluteVariableSpinBox(
        const moveit::core::VariableBounds& var_bounds);

    bool isVariableAngle(int vidx) const;

private Q_SLOTS:

    void updateRobotModel();
    void updateRobotState();

    // called when the item in the combo box changes
    void setJointGroup(const QString& group_name);

    void setJointVariableFromSpinBox(double value);
};

} // namespace sbpl_interface

#endif
