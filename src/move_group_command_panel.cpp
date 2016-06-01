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
    m_planner_name_combo_box(nullptr),
    m_planner_id_combo_box(nullptr),
    m_joint_tol_spinbox(nullptr),
    m_pos_tol_spinbox(nullptr),
    m_rot_tol_spinbox(nullptr),
    m_workspace_frame_combo_box(nullptr),
    m_workspace_min_x_spinbox(nullptr),
    m_workspace_min_y_spinbox(nullptr),
    m_workspace_min_z_spinbox(nullptr),
    m_workspace_max_x_spinbox(nullptr),
    m_workspace_max_y_spinbox(nullptr),
    m_workspace_max_z_spinbox(nullptr)
{
    setupGUI();

    // wait for a robot model to be loaded or for the robot's state to change
    connect(m_model.get(), SIGNAL(robotLoaded()),
            this, SLOT(updateRobot()));
    connect(m_model.get(), SIGNAL(robotStateChanged()),
            this, SLOT(syncRobot()));
    connect(m_model.get(), SIGNAL(configChanged()),
            this, SLOT(syncModelConfig()));
    connect(m_model.get(), SIGNAL(availableFramesUpdated()),
            this, SLOT(updateTransforms()));

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
    m_model->load(config);
}

void MoveGroupCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    ROS_INFO("Saving config for '%s'", this->getName().toStdString().c_str());
    m_model->save(config);
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
    const std::string& robot_description = m_model->robotDescription();
    if (m_robot_description_line_edit->text().toStdString() !=
        robot_description)
    {
        m_robot_description_line_edit->setText(
                QString::fromStdString(robot_description));
    }

    setupRobotGUI();
    syncRobot();
}

void MoveGroupCommandPanel::updateTransforms()
{
    QString workspace_frame = m_workspace_frame_combo_box->currentText();

    m_workspace_frame_combo_box->clear();
    for (const std::string& frame : m_model->availableFrames()) {
        m_workspace_frame_combo_box->addItem(QString::fromStdString(frame));
    }

    if (workspace_frame.isEmpty() && m_workspace_frame_combo_box->count() > 0) {
        // select the first item
        m_workspace_frame_combo_box->setCurrentIndex(0);
    }
    else {
        // select the previously selected item
        for (int i = 0; i < m_workspace_frame_combo_box->count(); ++i) {
            const std::string text =
                    m_workspace_frame_combo_box->itemText(i).toStdString();
            if (text == workspace_frame.toStdString()) {
                m_workspace_frame_combo_box->setCurrentIndex(i);
                break;
            }
        }
    }
}

void MoveGroupCommandPanel::syncRobot()
{
    syncSpinBoxes();
    updateRobotVisualization();
    Q_EMIT configChanged();
}

void MoveGroupCommandPanel::syncModelConfig()
{
    syncPlannerNameComboBox();
    syncPlannerIdComboBox();
    syncNumPlanningAttemptsSpinBox();
    syncAllowedPlanningTimeSpinBox();
    syncPlanningJointGroupComboBox();
    syncGoalPositionToleranceSpinBox();
    syncGoalOrientationToleranceSpinBox();
    syncGoalJointToleranceSpinBox();
    syncWorkspaceWidgets();
    Q_EMIT configChanged();
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

    // Planner Settings
    QGroupBox* planner_settings_group = new QGroupBox(tr("Planner Settings"));
    QGridLayout* planner_settings_layout = new QGridLayout;

    QLabel* planner_name_label = new QLabel(tr("Name:"));
    QLabel* planner_id_label = new QLabel(tr("ID:"));

    m_planner_name_combo_box = new QComboBox;
    m_planner_id_combo_box = new QComboBox;

    // add all planner names as items in the combo box
    const auto& planner_interfaces = m_model->plannerInterfaces();
    for (const auto& planner_interface : planner_interfaces) {
        const std::string& planner_name = planner_interface.name;
        m_planner_name_combo_box->addItem(QString::fromStdString(planner_name));
    }

    // add all planner ids part of the current planner as items in the combo box
    const std::string planner_name = m_model->plannerName();
    if (planner_name != "UNKNOWN") {
        size_t pidx;
        for (pidx = 0; pidx < planner_interfaces.size(); ++pidx) {
            if (planner_interfaces[pidx].name == planner_name) {
                break;
            }
        }

        for (const auto& planner_id : planner_interfaces[pidx].planner_ids) {
            m_planner_id_combo_box->addItem(QString::fromStdString(planner_id));
        }
    }

    syncPlannerNameComboBox();
    syncPlannerIdComboBox();

    QLabel* num_attempts_label = new QLabel(tr("Num Attempts"));
    m_num_planning_attempts_spinbox = new QSpinBox;
    m_num_planning_attempts_spinbox->setMinimum(1);
    m_num_planning_attempts_spinbox->setMaximum(100);
    m_num_planning_attempts_spinbox->setWrapping(false);
    syncNumPlanningAttemptsSpinBox();

    QLabel* allowed_planning_time_label = new QLabel(tr("Allowed Time (s)"));
    m_allowed_planning_time_spinbox = new QDoubleSpinBox;
    m_allowed_planning_time_spinbox->setMinimum(1.0);
    m_allowed_planning_time_spinbox->setMaximum(120.0);
    m_allowed_planning_time_spinbox->setSingleStep(1.0);
    m_allowed_planning_time_spinbox->setWrapping(false);
    syncAllowedPlanningTimeSpinBox();

    planner_settings_layout->addWidget(planner_name_label,              0, 0);
    planner_settings_layout->addWidget(m_planner_name_combo_box,        0, 1);
    planner_settings_layout->addWidget(planner_id_label,                1, 0);
    planner_settings_layout->addWidget(m_planner_id_combo_box,          1, 1);
    planner_settings_layout->addWidget(num_attempts_label,              2, 0);
    planner_settings_layout->addWidget(m_num_planning_attempts_spinbox, 2, 1);
    planner_settings_layout->addWidget(allowed_planning_time_label,     3, 0);
    planner_settings_layout->addWidget(m_allowed_planning_time_spinbox, 3, 1);

    connect(m_planner_name_combo_box, SIGNAL(currentIndexChanged(const QString&)),
            this, SLOT(setCurrentPlanner(const QString&)));
    connect(m_planner_id_combo_box, SIGNAL(currentIndexChanged(const QString&)),
            this, SLOT(setCurrentPlannerID(const QString&)));
    connect(m_num_planning_attempts_spinbox, SIGNAL(valueChanged(int)),
            m_model.get(), SLOT(setNumPlanningAttempts(int)));
    connect(m_allowed_planning_time_spinbox, SIGNAL(valueChanged(double)),
            m_model.get(), SLOT(setAllowedPlanningTime(double)));

    planner_settings_group->setLayout(planner_settings_layout);
    main_layout->addWidget(planner_settings_group);

    // Goal Constraints
    QGroupBox* goal_constraints_group = new QGroupBox(tr("Goal Constraints"));
    QGridLayout* goal_constraints_layout = new QGridLayout;

    QLabel* joint_tol_label = new QLabel(tr("Joint Tolerance (deg):"));

    m_joint_tol_spinbox = new QDoubleSpinBox;
    m_joint_tol_spinbox->setMinimum(-180.0);
    m_joint_tol_spinbox->setMaximum( 180.0);
    m_joint_tol_spinbox->setSingleStep(1.0);
    m_joint_tol_spinbox->setWrapping(false);
    syncGoalJointToleranceSpinBox();

    QLabel* pos_tol_label = new QLabel(tr("Position Tolerance (m):"));

    m_pos_tol_spinbox = new QDoubleSpinBox;
    m_pos_tol_spinbox->setMinimum(-1.0);
    m_pos_tol_spinbox->setMaximum( 1.0);
    m_pos_tol_spinbox->setSingleStep(0.01);
    m_pos_tol_spinbox->setWrapping(false);
    syncGoalPositionToleranceSpinBox();

    QLabel* rot_tol_label = new QLabel(tr("Orientation Tolerance (deg):"));

    m_rot_tol_spinbox = new QDoubleSpinBox;
    m_rot_tol_spinbox->setMinimum(0.0);
    m_rot_tol_spinbox->setMaximum(180.0);
    m_rot_tol_spinbox->setSingleStep(1.0);
    m_rot_tol_spinbox->setWrapping(false);
    syncGoalOrientationToleranceSpinBox();

    QGroupBox* workspace_group = new QGroupBox(tr("Workspace Parameters"));
    QGridLayout* workspace_layout = new QGridLayout;

    m_workspace_frame_combo_box = new QComboBox;
    for (const std::string& frame : m_model->availableFrames()) {
        m_workspace_frame_combo_box->addItem(QString::fromStdString(frame));
    }

    m_workspace_min_x_spinbox = new QDoubleSpinBox;
    m_workspace_min_x_spinbox->setMinimum(-5.0);
    m_workspace_min_x_spinbox->setMaximum(5.0);
    m_workspace_min_x_spinbox->setSingleStep(0.01);
    m_workspace_min_x_spinbox->setWrapping(false);

    m_workspace_min_y_spinbox = new QDoubleSpinBox;
    m_workspace_min_y_spinbox->setMinimum(-5.0);
    m_workspace_min_y_spinbox->setMaximum(5.0);
    m_workspace_min_y_spinbox->setSingleStep(0.01);
    m_workspace_min_y_spinbox->setWrapping(false);

    m_workspace_min_z_spinbox = new QDoubleSpinBox;
    m_workspace_min_z_spinbox->setMinimum(-5.0);
    m_workspace_min_z_spinbox->setMaximum(5.0);
    m_workspace_min_z_spinbox->setSingleStep(0.01);
    m_workspace_min_z_spinbox->setWrapping(false);

    m_workspace_max_x_spinbox = new QDoubleSpinBox;
    m_workspace_max_x_spinbox->setMinimum(-5.0);
    m_workspace_max_x_spinbox->setMaximum(5.0);
    m_workspace_max_x_spinbox->setSingleStep(0.01);
    m_workspace_max_x_spinbox->setWrapping(false);

    m_workspace_max_y_spinbox = new QDoubleSpinBox;
    m_workspace_max_y_spinbox->setMinimum(-5.0);
    m_workspace_max_y_spinbox->setMaximum(5.0);
    m_workspace_max_y_spinbox->setSingleStep(0.01);
    m_workspace_max_y_spinbox->setWrapping(false);

    m_workspace_max_z_spinbox = new QDoubleSpinBox;
    m_workspace_max_z_spinbox->setMinimum(-5.0);
    m_workspace_max_z_spinbox->setMaximum(5.0);
    m_workspace_max_z_spinbox->setSingleStep(0.01);
    m_workspace_max_z_spinbox->setWrapping(false);

    syncWorkspaceWidgets();

    workspace_layout->addWidget(new QLabel(tr("Frame:")),       0, 0);
    workspace_layout->addWidget(m_workspace_frame_combo_box,    0, 1, 1, 3);
    workspace_layout->addWidget(new QLabel(tr("Min Corner:")),  1, 0);
    workspace_layout->addWidget(m_workspace_min_x_spinbox,      1, 1);
    workspace_layout->addWidget(m_workspace_min_y_spinbox,      1, 2);
    workspace_layout->addWidget(m_workspace_min_z_spinbox,      1, 3);
    workspace_layout->addWidget(new QLabel(tr("Max Corner:")),  2, 0);
    workspace_layout->addWidget(m_workspace_max_x_spinbox,      2, 1);
    workspace_layout->addWidget(m_workspace_max_y_spinbox,      2, 2);
    workspace_layout->addWidget(m_workspace_max_z_spinbox,      2, 3);

    workspace_group->setLayout(workspace_layout);

    goal_constraints_layout->addWidget(pos_tol_label,       0, 0);
    goal_constraints_layout->addWidget(m_pos_tol_spinbox,   0, 1);
    goal_constraints_layout->addWidget(rot_tol_label,       1, 0);
    goal_constraints_layout->addWidget(m_rot_tol_spinbox,   1, 1);
    goal_constraints_layout->addWidget(joint_tol_label,     2, 0);
    goal_constraints_layout->addWidget(m_joint_tol_spinbox, 2, 1);
    goal_constraints_layout->addWidget(workspace_group,     3, 0, 1, 2);

    connect(m_joint_tol_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setGoalJointTolerance(double)));

    connect(m_pos_tol_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setGoalPositionTolerance(double)));

    connect(m_rot_tol_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setGoalOrientationTolerance(double)));

    connect(m_workspace_frame_combo_box, SIGNAL(currentIndexChanged(const QString&)),
            this, SLOT(setWorkspaceFrame(const QString&)));
    connect(m_workspace_min_x_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setWorkspaceMinX(double)));
    connect(m_workspace_min_y_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setWorkspaceMinY(double)));
    connect(m_workspace_min_z_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setWorkspaceMinZ(double)));
    connect(m_workspace_max_x_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setWorkspaceMaxX(double)));
    connect(m_workspace_max_y_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setWorkspaceMaxY(double)));
    connect(m_workspace_max_z_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setWorkspaceMaxZ(double)));

    goal_constraints_group->setLayout(goal_constraints_layout);
    main_layout->addWidget(goal_constraints_group);
}

void MoveGroupCommandPanel::setupRobotGUI()
{
    ROS_INFO("Setting up the Robot GUI");

    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();

    // set up combobox for selecting the active planning joint group
    m_joint_groups_combo_box = new QComboBox;

    // add all joint groups as items in the combobox
    for (size_t jgind = 0;
        jgind < robot_model->getJointModelGroupNames().size();
        ++jgind)
    {
        const std::string& jg_name =
                robot_model->getJointModelGroupNames()[jgind];
        m_joint_groups_combo_box->addItem(QString::fromStdString(jg_name));
    }

    syncPlanningJointGroupComboBox();

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
            this, SLOT(planToGoalPose()));

    m_move_to_position_button = new QPushButton(tr("Move to Position"));
    connect(m_move_to_position_button, SIGNAL(clicked()),
            this, SLOT(moveToGoalPose()));

    m_copy_current_state_button = new QPushButton(tr("Copy Current State"));
    connect(m_copy_current_state_button, SIGNAL(clicked()),
            this, SLOT(copyCurrentState()));

    QVBoxLayout* vlayout = qobject_cast<QVBoxLayout*>(layout());

    // Commands
    QGroupBox* commands_group_box = new QGroupBox(tr("Commands"));
    QVBoxLayout* commands_group_layout = new QVBoxLayout;

    commands_group_layout->addWidget(m_plan_to_position_button);
    commands_group_layout->addWidget(m_move_to_position_button);
    commands_group_layout->addWidget(m_copy_current_state_button);

    commands_group_box->setLayout(commands_group_layout);

    vlayout->insertWidget(vlayout->count(), m_joint_groups_combo_box);
    vlayout->insertWidget(vlayout->count(), m_var_cmd_widget);
    vlayout->insertWidget(vlayout->count(), commands_group_box);
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

void MoveGroupCommandPanel::syncPlannerNameComboBox()
{
    assert(m_model && m_planner_name_combo_box);
    // set the selected item to match the current planner
    const std::string planner_name = m_model->plannerName();
    for (int i = 0; i < m_planner_name_combo_box->count(); ++i) {
        if (m_planner_name_combo_box->itemText(i).toStdString() ==
                planner_name)
        {
            m_planner_name_combo_box->setCurrentIndex(i);
            break;
        }
    }
}

void MoveGroupCommandPanel::syncPlannerIdComboBox()
{
    assert(m_model && m_planner_id_combo_box);
    // set the selected item to match the current planner id
    const std::string planner_id = m_model->plannerID();
    for (int i = 0; i < m_planner_id_combo_box->count(); ++i) {
        if (m_planner_id_combo_box->itemText(i).toStdString() == planner_id) {
            m_planner_id_combo_box->setCurrentIndex(i);
            break;
        }
    }
}

void MoveGroupCommandPanel::syncNumPlanningAttemptsSpinBox()
{
    assert(m_model && m_num_planning_attempts_spinbox);
    m_num_planning_attempts_spinbox->setValue(m_model->numPlanningAttempts());
}

void MoveGroupCommandPanel::syncAllowedPlanningTimeSpinBox()
{
    assert(m_model && m_allowed_planning_time_spinbox);
    m_allowed_planning_time_spinbox->setValue(m_model->allowedPlanningTime());
}

void MoveGroupCommandPanel::syncPlanningJointGroupComboBox()
{
    assert(m_model && m_joint_groups_combo_box);
    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();
    if (!robot_model) {
        // TODO: set to empty selection?
        return;
    }

    // set the selected item to match the active planning joint group
    if (!robot_model->getJointModelGroups().empty()) {
        const int ajg_idx = m_joint_groups_combo_box->findText(
                QString::fromStdString(m_model->planningJointGroupName()));
        m_joint_groups_combo_box->setCurrentIndex(ajg_idx);
    }
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

void MoveGroupCommandPanel::syncGoalPositionToleranceSpinBox()
{
    assert(m_model && m_pos_tol_spinbox);
    m_pos_tol_spinbox->setValue(m_model->goalPositionTolerance());
}

void MoveGroupCommandPanel::syncGoalOrientationToleranceSpinBox()
{
    assert(m_model && m_rot_tol_spinbox);
    m_rot_tol_spinbox->setValue(m_model->goalOrientationTolerance());
}

void MoveGroupCommandPanel::syncGoalJointToleranceSpinBox()
{
    assert(m_model && m_joint_tol_spinbox);
    m_joint_tol_spinbox->setValue(m_model->goalJointTolerance());
}

void MoveGroupCommandPanel::syncWorkspaceWidgets()
{
    assert(m_model &&
            m_workspace_min_x_spinbox && m_workspace_max_x_spinbox &&
            m_workspace_min_y_spinbox && m_workspace_max_y_spinbox &&
            m_workspace_min_z_spinbox && m_workspace_max_z_spinbox);
    bool found = false;
    for (int i = 0; i < m_workspace_frame_combo_box->count(); ++i) {
        if (m_workspace_frame_combo_box->itemText(i).toStdString() ==
                m_model->workspace().header.frame_id)
        {
            m_workspace_frame_combo_box->setCurrentIndex(i);
            found = true;
            break;
        }
    }
    if (!found) {
        m_workspace_frame_combo_box->setCurrentIndex(-1);
    }
    m_workspace_min_x_spinbox->setValue(m_model->workspace().min_corner.x);
    m_workspace_min_y_spinbox->setValue(m_model->workspace().min_corner.y);
    m_workspace_min_z_spinbox->setValue(m_model->workspace().min_corner.z);
    m_workspace_max_x_spinbox->setValue(m_model->workspace().max_corner.x);
    m_workspace_max_y_spinbox->setValue(m_model->workspace().max_corner.y);
    m_workspace_max_z_spinbox->setValue(m_model->workspace().max_corner.z);
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
    m_model->setPlanningJointGroup(joint_group_name.toStdString());
}

void MoveGroupCommandPanel::planToGoalPose()
{
    std::string current_joint_group =
            m_joint_groups_combo_box->currentText().toStdString();
    if (!m_model->planToGoalPose(current_joint_group)) {
        ROS_ERROR("This should be a message box");
    }
}

void MoveGroupCommandPanel::moveToGoalPose()
{
    std::string curr_joint_group =
            m_joint_groups_combo_box->currentText().toStdString();
    if (!m_model->moveToGoalPose(curr_joint_group)) {
        ROS_ERROR("This should also be a message box");
    }
}

void MoveGroupCommandPanel::copyCurrentState()
{
    m_model->copyCurrentState();
}

void MoveGroupCommandPanel::setGoalJointTolerance(double tol_deg)
{
    m_model->setGoalJointTolerance(tol_deg);
}

void MoveGroupCommandPanel::setGoalPositionTolerance(double tol_m)
{
    m_model->setGoalPositionTolerance(tol_m);
}

void MoveGroupCommandPanel::setGoalOrientationTolerance(double tol_deg)
{
    m_model->setGoalOrientationTolerance(tol_deg);
}

void MoveGroupCommandPanel::setCurrentPlanner(const QString& name)
{
    m_model->setPlannerName(name.toStdString());
}

void MoveGroupCommandPanel::setCurrentPlannerID(const QString& id)
{
    m_model->setPlannerID(id.toStdString());
}

void MoveGroupCommandPanel::setWorkspaceFrame(const QString& frame)
{
    moveit_msgs::WorkspaceParameters params = m_model->workspace();
    params.header.frame_id = frame.toStdString();
    m_model->setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMinX(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model->workspace();
    params.min_corner.x = value;
    m_model->setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMinY(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model->workspace();
    params.min_corner.y = value;
    m_model->setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMinZ(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model->workspace();
    params.min_corner.z = value;
    m_model->setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMaxX(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model->workspace();
    params.max_corner.x = value;
    m_model->setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMaxY(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model->workspace();
    params.max_corner.y = value;
    m_model->setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMaxZ(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model->workspace();
    params.max_corner.z = value;
    m_model->setWorkspace(params);
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
