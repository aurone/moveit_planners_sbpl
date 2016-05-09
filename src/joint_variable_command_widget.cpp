#include "joint_variable_command_widget.h"

// system includes
#include <ros/console.h>
#include <sbpl_geometry_utils/utils.h>

// module includes
#include "move_group_command_model.h"

namespace sbpl_interface {

JointVariableCommandWidget::JointVariableCommandWidget(
    MoveGroupCommandModel* model,
    QWidget* parent)
:
    Base(parent),
    m_model(model)
{
    setWidgetResizable(true);

    auto robot_model = m_model->robotModel();

    QVBoxLayout* scroll_area_layout = new QVBoxLayout;

    QWidget* joint_commands_widget = new QWidget(this);
    QGridLayout* joint_commands_layout = new QGridLayout;

    const size_t num_vars = robot_model->getVariableCount();
    const std::vector<std::string>& var_names =
            robot_model->getVariableNames();

    // create a (label, spinbox) combo for each joint variable
    // create bijection between spinboxes and joint variable indices
    for (size_t vind = 0; vind < num_vars; ++vind) {
        const std::string& var_name = var_names[vind];
        const auto& var_bounds = robot_model->getVariableBounds(var_name);

        // set the bounds, step, and wrapping on the spinbox
        const moveit::core::JointModel* jm =
                robot_model->getJointOfVariable(var_name);

        QLabel* var_label = new QLabel(QString::fromStdString(var_name));
        QDoubleSpinBox* var_spinbox = setupSpinBoxFor(var_name, var_bounds, *jm);

        m_spinbox_to_vind.insert(std::make_pair(var_spinbox, vind));
        m_vind_to_spinbox.push_back(var_spinbox);
        m_vind_to_label.push_back(var_label);

//        QHBoxLayout* form_layout = new QHBoxLayout;
//        form_layout->addWidget(var_label);
//        form_layout->addWidget(var_spinbox);
//        joint_commands_layout->addLayout(form_layout);

//        joint_commands_layout->addWidget(var_label, vind, 0);
//        joint_commands_layout->addWidget(var_spinbox, vind, 1);
//        var_label->hide();
//        var_spinbox->hide();
    }

    joint_commands_widget->setLayout(joint_commands_layout);
    setLayout(scroll_area_layout);

    setWidget(joint_commands_widget);
}

JointVariableCommandWidget::~JointVariableCommandWidget()
{
    // TODO: are the spinboxes/labels that are not currently attached to the
    // widget deleted when this entire widget is deleted? Probably not, and in
    // that case we may have to delete them manually
}

int JointVariableCommandWidget::spinboxToVariableIndex(
    QDoubleSpinBox* spinbox) const
{
    auto it = m_spinbox_to_vind.find(spinbox);
    if (it == m_spinbox_to_vind.end()) {
        ROS_WARN("setJointVariableFromSpinBox not called from a registered spinbox");
        return -1;
    }

    int vind = it->second;
    return vind;
}

void JointVariableCommandWidget::displayJointGroupCommands(
    const std::string& joint_group_name)
{
    QGridLayout* glayout = qobject_cast<QGridLayout*>(widget()->layout());
    if (!glayout) {
        ROS_ERROR("JointVariableCommandWidget layout is not a QGridLayout");
        return;
    }

    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();
    if (!robot_model) {
        ROS_ERROR("Robot Model is null");
        return;
    }

    const moveit::core::JointModelGroup* jmg =
            robot_model->getJointModelGroup(joint_group_name);
    if (!jmg) {
        ROS_ERROR("Joint Group '%s' does not exist within Robot Model '%s'", joint_group_name.c_str(), robot_model->getName().c_str());
        return;
    }

    ROS_INFO("Removing everything from layout");

    // remove everything from the layout
    while (!glayout->isEmpty()) {
        QLayoutItem* item = glayout->takeAt(0);
        if (item->widget()) {
            item->widget()->hide();
            glayout->removeWidget(item->widget());
        }
        else {
            ROS_WARN("Here there be layouts?!");
            glayout->removeItem(item);
            delete item;
        }
    }

    ROS_INFO("Adding joint group controls to layout");

    int num_added = 0;
    const auto& joint_models = jmg->getJointModels();
    for (const moveit::core::JointModel* joint_model : joint_models) {
        const auto& var_names = joint_model->getVariableNames();
        for (const std::string& var_name : var_names) {
            int vind = robot_model->getVariableIndex(var_name);
            ROS_INFO("Adding spinbox %p and label %p to layout", m_vind_to_spinbox[vind], m_vind_to_label[vind]);
            glayout->addWidget(m_vind_to_label[vind], num_added, 0);
            glayout->addWidget(m_vind_to_spinbox[vind], num_added, 1);
            m_vind_to_spinbox[vind]->show();
            m_vind_to_label[vind]->show();
            ++num_added;
        }
    }

//    for (int i = 0; i < glayout->count(); ++i) {
//        glayout->itemAt(i)->invalidate();
//    }
//
    glayout->invalidate();
    layout()->invalidate();
//    glayout->activate();
//    layout()->activate();
    glayout->update();
    layout()->update();

    ROS_INFO("Layout contains %d controls", glayout->count());
}

QDoubleSpinBox* JointVariableCommandWidget::setupSpinBoxFor(
    const std::string& var_name,
    const moveit::core::VariableBounds& var_bounds,
    const moveit::core::JointModel& jm)
{
    QDoubleSpinBox* var_spinbox = new QDoubleSpinBox;

    if (jm.getType() == moveit::core::JointModel::REVOLUTE) {
        if (var_bounds.position_bounded_) {
            var_spinbox->setMinimum(
                    sbpl::utils::ToDegrees(var_bounds.min_position_));
            var_spinbox->setMaximum(
                    sbpl::utils::ToDegrees(var_bounds.max_position_));
            var_spinbox->setSingleStep(1.0);
            var_spinbox->setWrapping(false);
        }
        else {
            var_spinbox->setMinimum(0.0);
            var_spinbox->setMaximum(359.0);
            var_spinbox->setSingleStep(1.0);
            var_spinbox->setWrapping(true);
        }
    }
    else if (jm.getType() == moveit::core::JointModel::PRISMATIC) {
        if (var_bounds.position_bounded_) {
            var_spinbox->setMinimum(var_bounds.min_position_);
            var_spinbox->setMaximum(var_bounds.max_position_);
            const double step =
                    (var_bounds.max_position_ - var_bounds.min_position_) /
                    100.0;
            // TODO: compute the number of decimals required to display a
            // change in the joint variable or round the step up to the
            // nearest number of decimals desired to be displayed
            var_spinbox->setDecimals(3);
            var_spinbox->setSingleStep(step);
            ROS_INFO("Single step for bounded prismatic joint: %f", step);
            var_spinbox->setWrapping(false);
        }
        else {
            ROS_WARN("A prismatic joint without bounds?! This is somewhat unexpected");
            var_spinbox->setMinimum(-100.0);
            var_spinbox->setMaximum(100.0);
            var_spinbox->setSingleStep(2.0);
            var_spinbox->setWrapping(false);
        }
    }
    else if (jm.getType() == moveit::core::JointModel::PLANAR) {
        if (var_bounds.position_bounded_) {
            ROS_WARN("A planar joint with bounds?! Assuming a position variable x or y");
            ROS_WARN("  Joint: %s", jm.getName().c_str());
            ROS_WARN("  Variable: %s", var_name.c_str());
            ROS_WARN("  Min Position: %f", var_bounds.min_position_);
            ROS_WARN("  Max Position: %f", var_bounds.max_position_);
            var_spinbox->setMinimum(-1000.0);
            var_spinbox->setMaximum(1000.0);
            var_spinbox->setSingleStep(0.01);
            var_spinbox->setWrapping(false);
        }
        else {
            ROS_WARN("A planar joint without bounds. Assuming an orientation variable yaw");
            var_spinbox->setMinimum(0.0);
            var_spinbox->setMaximum(359.0);
            var_spinbox->setSingleStep(1.0);
            var_spinbox->setWrapping(true);
        }
    }
    else if (jm.getType() == moveit::core::JointModel::FLOATING) {
        if (var_bounds.position_bounded_) {
            ROS_WARN("A floating joint with bounds?! Assuming a position variable x, y, or z");
            var_spinbox->setMinimum(-1000.0);
            var_spinbox->setMaximum(1000.0);
            var_spinbox->setSingleStep(0.01);
            var_spinbox->setWrapping(false);
        }
        else {
            ROS_WARN("A planar joint without bounds. Assuming an orientation variable roll, pitch, or yaw");
            var_spinbox->setMinimum(-360.0);
            var_spinbox->setMaximum(360.0);
            var_spinbox->setSingleStep(1.0);
            var_spinbox->setWrapping(true);
        }
    }
    else {
        ROS_WARN("Unrecognized joint type");
        var_spinbox->setMinimum(0.0);
        var_spinbox->setMaximum(100.0);
        var_spinbox->setSingleStep(1.0);
        var_spinbox->setWrapping(false);
    }

    return var_spinbox;
}

} // namespace sbpl_interface
