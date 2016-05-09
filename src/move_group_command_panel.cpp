#include "move_group_command_panel.h"

// system includes
#include <sbpl_geometry_utils/utils.h>
#include <visualization_msgs/MarkerArray.h>

// module includes
#include "move_group_command_model.h"
#include "joint_variable_command_widget.h"

namespace sbpl_interface {

MoveGroupCommandPanel::MoveGroupCommandPanel(QWidget* parent) :
    rviz::Panel(parent),
    m_nh(),
    m_model(new MoveGroupCommandModel),
    m_robot_description_line_edit(nullptr),
    m_load_robot_button(nullptr),
    m_joint_groups_combo_box(nullptr),
    m_arm_commands_group(nullptr),
    m_marker_pub(),
    m_var_cmd_widget(nullptr),
    m_table_x_spinbox(nullptr),
    m_table_y_spinbox(nullptr),
    m_table_z_spinbox(nullptr)
{
    setupGUI();

    // wait for a robot model to be loaded or for the robot's state to change
    connect(m_model.get(), SIGNAL(robotLoaded()),
            this, SLOT(updateRobot()));
    connect(m_model.get(), SIGNAL(robotStateChanged()),
            this, SLOT(syncRobot()));

    m_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 5);
}

MoveGroupCommandPanel::~MoveGroupCommandPanel()
{
}

void MoveGroupCommandPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);

    ROS_INFO("Loading config for '%s'", this->getName().toStdString().c_str());

    QString robot_description;
    config.mapGetString("robot_description", &robot_description);

    ROS_INFO("Robot Description: %s", robot_description.toStdString().c_str());

    if (m_model->loadRobot(robot_description.toStdString())) {
        m_robot_description_line_edit->setText(robot_description);
    }
}

void MoveGroupCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);

    ROS_INFO("Saving config for '%s'", this->getName().toStdString().c_str());

    config.mapSetValue(
            "robot_description",
            QString::fromStdString(m_model->robotDescription()));

    // TODO: save the state of the MoveGroupCommandModel
}

void MoveGroupCommandPanel::loadRobot()
{
    std::string user_robot_description =
            m_robot_description_line_edit->text().toStdString();

    if (user_robot_description.empty()) {
        QMessageBox::information(
                this,
                tr("Robot Description"),
                tr("Please enter a valid ROS parameter for the URDF"));
        return;
    }

    if (!m_model->loadRobot(user_robot_description)) {
        QMessageBox::warning(
                this,
                tr("Robot Description"),
                tr("Failed to load robot from robot description to '%1'")
                        .arg(QString::fromStdString(user_robot_description)));
    }
}

void MoveGroupCommandPanel::updateRobot()
{
    setupRobotGUI();
    syncRobot();
}

void MoveGroupCommandPanel::syncRobot()
{
    syncSpinBoxes();
    updateRobotVisualization();
}

void MoveGroupCommandPanel::setupGUI()
{
    ROS_INFO("Setting up the baseline GUI");

    QVBoxLayout* main_layout = new QVBoxLayout;

    // general settings
    QGroupBox* general_settings_group = new QGroupBox(tr("General Settings"));
    QVBoxLayout* general_settings_layout = new QVBoxLayout;
    QLabel* robot_description_label = new QLabel(tr("Robot Description:"));

    QHBoxLayout* robot_description_layout = new QHBoxLayout;
    m_robot_description_line_edit = new QLineEdit;
    m_load_robot_button = new QPushButton(tr("Load Robot"));
    robot_description_layout->addWidget(m_robot_description_line_edit);
    robot_description_layout->addWidget(m_load_robot_button);

    general_settings_layout->addWidget(robot_description_label);
    general_settings_layout->addLayout(robot_description_layout);
    general_settings_group->setLayout(general_settings_layout);

    main_layout->addWidget(general_settings_group);
    setLayout(main_layout);

    connect(m_load_robot_button, SIGNAL(clicked()), this, SLOT(loadRobot()));

    if (m_model->isRobotLoaded()) {
        setupRobotGUI();
    }

//    main_layout->addStretch();
}

void MoveGroupCommandPanel::setupRobotGUI()
{
    ROS_INFO("Setting up the Robot GUI");

    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();

    // add all joint groups as items in a combobox
    m_joint_groups_combo_box = new QComboBox;
    // set up combobox for choosing joint group to modify
    for (size_t jgind = 0;
        jgind < robot_model->getJointModelGroupNames().size();
        ++jgind)
    {
        const std::string& jg_name =
                robot_model->getJointModelGroupNames()[jgind];
        m_joint_groups_combo_box->addItem(QString::fromStdString(jg_name));
    }

    // NOTE: the first item added to the combobox will become the value of the
    // combobox

    connect(m_joint_groups_combo_box,
            SIGNAL(currentIndexChanged(const QString&)),
            this,
            SLOT(setJointGroup(const QString&)));

    m_var_cmd_widget = setupJointVariableCommandWidget();
    for (QDoubleSpinBox* spinbox : m_var_cmd_widget->spinboxes()) {
        connect(spinbox, SIGNAL(valueChanged(double)),
                this, SLOT(setJointVariableFromSpinBox(double)));
    }

    updateJointVariableCommandWidget(
        m_joint_groups_combo_box->currentText().toStdString());

    m_plan_to_position_button = new QPushButton(tr("Plan to Position"));
    connect(m_plan_to_position_button, SIGNAL(clicked()),
            this, SLOT(planToPosition()));

    m_copy_current_state_button = new QPushButton(tr("Copy Current State"));
    connect(m_copy_current_state_button, SIGNAL(clicked()),
            this, SLOT(copyCurrentState()));

    m_table_x_spinbox = new QDoubleSpinBox;
    m_table_x_spinbox->setMinimum(-10.0);
    m_table_x_spinbox->setMaximum( 10.0);
    m_table_x_spinbox->setSingleStep(0.10);
    m_table_x_spinbox->setWrapping(false);
    connect(m_table_x_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setTableX(double)));

    m_table_y_spinbox = new QDoubleSpinBox;
    m_table_y_spinbox->setMinimum(-10.0);
    m_table_y_spinbox->setMaximum( 10.0);
    m_table_y_spinbox->setSingleStep(0.10);
    m_table_y_spinbox->setWrapping(false);
    connect(m_table_y_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setTableY(double)));

    m_table_z_spinbox = new QDoubleSpinBox;
    m_table_z_spinbox->setMinimum(-10.0);
    m_table_z_spinbox->setMaximum( 10.0);
    m_table_z_spinbox->setSingleStep(0.10);
    m_table_z_spinbox->setWrapping(false);
    connect(m_table_z_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setTableZ(double)));

    QVBoxLayout* vlayout = qobject_cast<QVBoxLayout*>(layout());
    vlayout->insertWidget(vlayout->count(), m_joint_groups_combo_box);
    vlayout->insertWidget(vlayout->count(), m_var_cmd_widget);
    vlayout->insertWidget(vlayout->count(), m_plan_to_position_button);
    vlayout->insertWidget(vlayout->count(), m_copy_current_state_button);
    vlayout->insertWidget(vlayout->count(), m_table_x_spinbox);
    vlayout->insertWidget(vlayout->count(), m_table_y_spinbox);
    vlayout->insertWidget(vlayout->count(), m_table_z_spinbox);
    vlayout->addStretch();
}

JointVariableCommandWidget*
MoveGroupCommandPanel::setupJointVariableCommandWidget()
{
    return new JointVariableCommandWidget(m_model.get());
}

void MoveGroupCommandPanel::updateJointVariableCommandWidget(
    const std::string& joint_group_name)
{
    m_var_cmd_widget->displayJointGroupCommands(joint_group_name);
}

void MoveGroupCommandPanel::syncSpinBoxes()
{
    if (!m_model->isRobotLoaded()) {
        ROS_WARN("Robot not yet loaded");
        return;
    }

    auto robot_model = m_model->robotModel();
    auto robot_state = m_model->robotState();

    for (int i = 0; i < (int)robot_model->getVariableCount(); ++i) {
        QDoubleSpinBox* spinbox = m_var_cmd_widget->variableIndexToSpinBox(i);

        if (isVariableAngle(i)) {
            double value =
                    sbpl::utils::ToDegrees(robot_state->getVariablePosition(i));
            if (value != spinbox->value()) {
                spinbox->setValue(value);
            }
        }
        else {
            double value = robot_state->getVariablePosition(i);
            // this check is required because the internal value of the spinbox
            // may differ from the displayed value. Apparently, scrolling the
            // spinbox by a step less than the precision will update the
            // internal value, but calling setValue will ensure that the
            // internal value is the same as the value displayed. The absence
            // of this check can result in not being able to update a joint
            // variable
            if (value != spinbox->value()) {
                spinbox->setValue(value);
            }
        }
    }
}

void MoveGroupCommandPanel::updateRobotVisualization()
{
    ROS_DEBUG("Updating robot visualization");

    if (!m_model->isRobotLoaded()) {
        ROS_WARN("Robot not yet loaded");
        return;
    }

    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();
    moveit::core::RobotStateConstPtr robot_state = m_model->robotState();

    visualization_msgs::MarkerArray marr;
    robot_state->getRobotMarkers(marr, robot_model->getLinkModelNames());

    const std::string ns = robot_model->getName() + std::string("_phantom");
    int id = 0;
    for (auto& marker : marr.markers) {
        marker.mesh_use_embedded_materials = false;

        float r_base = 0.4f; // (float)100 / (float)255;
        float g_base = 0.4f; // (float)159 / (float)255;
        float b_base = 0.4f; // (float)237 / (float)255;
        boost::tribool valid = m_model->robotStateValidity();
        if (valid) {
            g_base = 1.0f;
        }
        else if (!valid) {
            r_base = 1.0f;
        }
        else {
            r_base = 1.0f;
            g_base = 1.0f;
        }

        marker.color.r = r_base;
        marker.color.g = g_base;
        marker.color.b = b_base;
        marker.color.a = 0.8f;
        marker.ns = ns;
        marker.id = id++;
    }

    m_marker_pub.publish(marr);
}

void MoveGroupCommandPanel::setJointVariableFromSpinBox(double value)
{
    QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(sender());
    if (!spinbox) {
        ROS_WARN("setJointVariableFromSpinBox not called from a spinbox");
        return;
    }

    int vind = m_var_cmd_widget->spinboxToVariableIndex(spinbox);
    if (vind == -1) {
        ROS_ERROR("setJointVariableFromSpinBox called from spinbox not associated with a joint variable");
        return;
    }

    ROS_DEBUG("Joint variable %d set to %f from spinbox", vind, value);

    if (isVariableAngle(vind)) {
        // convert to radians and assign
        m_model->setJointVariable(vind, sbpl::utils::ToRadians(value));
    }
    else {
        // assign without conversion
        m_model->setJointVariable(vind, value);
    }
}

void MoveGroupCommandPanel::setJointGroup(const QString& joint_group_name)
{
    updateJointVariableCommandWidget(joint_group_name.toStdString());
}

void MoveGroupCommandPanel::planToPosition()
{
    std::string current_joint_group =
            m_joint_groups_combo_box->currentText().toStdString();
    m_model->planToPosition(current_joint_group);
}

void MoveGroupCommandPanel::copyCurrentState()
{
    m_model->copyCurrentState();
}

void MoveGroupCommandPanel::setTableX(double x)
{

}

void MoveGroupCommandPanel::setTableY(double y)
{

}

void MoveGroupCommandPanel::setTableZ(double z)
{

}

bool MoveGroupCommandPanel::isVariableAngle(int vind) const
{
    auto robot_model = m_model->robotModel();
    if (!robot_model) {
        ROS_WARN("Asking whether variable %d in uninitialized robot is an angle", vind);
        return false;
    }

    const moveit::core::JointModel* jm = robot_model->getJointOfVariable(vind);

    const std::string& var_name = robot_model->getVariableNames()[vind];

    const auto& var_bounds = jm->getVariableBounds(var_name);

    return (jm->getType() == moveit::core::JointModel::REVOLUTE ||
        (
            jm->getType() == moveit::core::JointModel::PLANAR &&
            !var_bounds.position_bounded_
        ) ||
        (
            jm->getType() == moveit::core::JointModel::FLOATING &&
            !var_bounds.position_bounded_
        ));
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbpl_interface::MoveGroupCommandPanel, rviz::Panel)
