#include "move_group_command_panel.h"

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <leatherman/viz.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <moveit_planners_sbpl/interface/joint_variable_command_widget.h>

// module includes
#include "move_group_command_model.h"

namespace sbpl_interface {

static const char* LOG = "move_group_command_panel";

MoveGroupCommandPanel::MoveGroupCommandPanel(QWidget* parent) :
    rviz::Panel(parent),
    m_ik_cmd_marker(m_model.getRobotCommandModel())
{
    m_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 5);

    setupGUI();

    // wait for a robot model to be loaded or for the robot's state to change
    connect(&m_model, SIGNAL(robotLoaded()), this, SLOT(updateRobot()));

    // NOTE: connect to m_model's robotStateChanged() signal instead of
    // directly to its RobotCommandModel's so that the signal is interrupted and
    // validity checking is run before visualizing the state of the robot
    connect(&m_model, SIGNAL(robotStateChanged()), this, SLOT(syncRobot()));

    connect(&m_model, SIGNAL(configChanged()), this, SLOT(syncModelConfig()));
    connect(&m_model, SIGNAL(availableFramesUpdated()),
            this, SLOT(updateTransforms()));
}

MoveGroupCommandPanel::~MoveGroupCommandPanel()
{
}

void MoveGroupCommandPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    ROS_INFO("Loading config for '%s'", this->getName().toStdString().c_str());
    m_model.load(config);
}

void MoveGroupCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    ROS_INFO("Saving config for '%s'", this->getName().toStdString().c_str());
    m_model.save(config);
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

    if (!m_model.loadRobot(user_robot_description)) {
        QMessageBox::warning(
                this,
                tr("Robot Description"),
                tr("Failed to load robot from robot description to '%1'")
                        .arg(QString::fromStdString(user_robot_description)));
    }
}

void MoveGroupCommandPanel::updateRobot()
{
    const auto& robot_description = m_model.robotDescription();
    if (m_robot_description_line_edit->text().toStdString() !=
        robot_description)
    {
        m_robot_description_line_edit->setText(
                QString::fromStdString(robot_description));
    }

    syncRobot();
}

void MoveGroupCommandPanel::updateTransforms()
{
    QString workspace_frame = m_workspace_frame_combo_box->currentText();

    m_workspace_frame_combo_box->clear();
    for (const std::string& frame : m_model.availableFrames()) {
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
    updateRobotVisualization();
    Q_EMIT configChanged();
}

void MoveGroupCommandPanel::syncModelConfig()
{
    syncPlannerNameComboBox();
    syncPlannerIdComboBox();
    syncNumPlanningAttemptsSpinBox();
    syncAllowedPlanningTimeSpinBox();

    // TODO: need to determine the owner of the active planning joint group
    // variable so that updates can be propagated from it and synchronized in
    // the gui
    m_var_cmd_widget->setActiveJointGroup(m_model.planningJointGroupName());
    m_ik_cmd_marker.setActiveJointGroup(m_model.planningJointGroupName());

    syncGoalPositionToleranceSpinBox();
    syncGoalOrientationToleranceSpinBox();
    syncGoalJointToleranceSpinBox();

    syncWorkspaceWidgets();
    Q_EMIT configChanged();
}

void MoveGroupCommandPanel::setupGUI()
{
    ROS_INFO("Setting up the baseline GUI");

    QVBoxLayout* parent_layout = new QVBoxLayout;
    QScrollArea* scroll_area = new QScrollArea;
    QWidget* scroll_area_widget = new QWidget;
    QVBoxLayout* main_layout = new QVBoxLayout;

    QGroupBox* general_settings_group = setupGeneralSettingsGroup();
    QGroupBox* planner_settings_group = setupPlannerSettingsGroup();
    m_goal_constraints_group = setupGoalConstraintsGroup();

    ///////////////////////////////////////////////
    // Populate GUI with initial property values //
    ///////////////////////////////////////////////

    // add all planner names as items in the combo box
    const auto& planner_interfaces = m_model.plannerInterfaces();
    for (const auto& planner_interface : planner_interfaces) {
        const std::string& planner_name = planner_interface.name;
        m_planner_name_combo_box->addItem(QString::fromStdString(planner_name));
    }

    // add all planner ids part of the current planner as items in the combo box
    const std::string planner_name = m_model.plannerName();
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

//    syncPlannerNameComboBox();
//    syncPlannerIdComboBox();
//
//    syncNumPlanningAttemptsSpinBox();
//    syncAllowedPlanningTimeSpinBox();
//
//    syncGoalJointToleranceSpinBox();
//    syncGoalPositionToleranceSpinBox();
//    syncGoalOrientationToleranceSpinBox();

    for (auto& frame : m_model.availableFrames()) {
        m_workspace_frame_combo_box->addItem(QString::fromStdString(frame));
    }

//    syncWorkspaceWidgets();

    ////////////////////
    // End population //
    ////////////////////

    assert(!m_model.isRobotLoaded());

    QGroupBox* commands_group_box = setupCommandsGroup();

    // Put it all together
    main_layout->addWidget(general_settings_group);
    main_layout->addWidget(planner_settings_group);
    main_layout->addWidget(m_goal_constraints_group);
    main_layout->insertWidget(main_layout->count(), commands_group_box);
    scroll_area_widget->setLayout(main_layout);
    scroll_area->setWidget(scroll_area_widget);
    scroll_area->setWidgetResizable(true);
    parent_layout->addWidget(scroll_area);

    setLayout(parent_layout);

    // Connect Signals
    connect(m_load_robot_button, SIGNAL(clicked()), this, SLOT(loadRobot()));

    connect(m_planner_name_combo_box, SIGNAL(currentIndexChanged(const QString&)),
            this, SLOT(setCurrentPlanner(const QString&)));
    connect(m_planner_id_combo_box, SIGNAL(currentIndexChanged(const QString&)),
            this, SLOT(setCurrentPlannerID(const QString&)));
    connect(m_num_planning_attempts_spinbox, SIGNAL(valueChanged(int)),
            &m_model, SLOT(setNumPlanningAttempts(int)));
    connect(m_allowed_planning_time_spinbox, SIGNAL(valueChanged(double)),
            &m_model, SLOT(setAllowedPlanningTime(double)));

    connect(m_joint_tol_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setGoalJointTolerance(double)));

    connect(m_pos_tol_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setGoalPositionTolerance(double)));

    connect(m_rot_tol_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setGoalOrientationTolerance(double)));

    connect(m_var_cmd_widget, SIGNAL(updateActiveJointGroup(const std::string&)),
            &m_model, SLOT(setPlanningJointGroup(const std::string&)));

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

    connect(m_plan_to_position_button, SIGNAL(clicked()),
            this, SLOT(planToGoalPose()));
    connect(m_move_to_position_button, SIGNAL(clicked()),
            this, SLOT(moveToGoalPose()));
    connect(m_plan_to_configuration_button, SIGNAL(clicked()),
            this, SLOT(planToGoalConfiguration()));
    connect(m_move_to_configuration_button, SIGNAL(clicked()),
            this, SLOT(moveToGoalConfiguration()));
    connect(m_copy_current_state_button, SIGNAL(clicked()),
            this, SLOT(copyCurrentState()));
}

QGroupBox* MoveGroupCommandPanel::setupGeneralSettingsGroup()
{
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
    return general_settings_group;
}

QGroupBox* MoveGroupCommandPanel::setupPlannerSettingsGroup()
{
    QGroupBox* planner_settings_group = new QGroupBox(tr("Planner Settings"));
    QGridLayout* planner_settings_layout = new QGridLayout;

    QLabel* planner_name_label = new QLabel(tr("Name:"));
    QLabel* planner_id_label = new QLabel(tr("ID:"));

    m_planner_name_combo_box = new QComboBox;
    m_planner_id_combo_box = new QComboBox;

    QLabel* num_attempts_label = new QLabel(tr("Num Attempts"));
    m_num_planning_attempts_spinbox = new QSpinBox;
    m_num_planning_attempts_spinbox->setMinimum(1);
    m_num_planning_attempts_spinbox->setMaximum(100);
    m_num_planning_attempts_spinbox->setWrapping(false);

    QLabel* allowed_planning_time_label = new QLabel(tr("Allowed Time (s)"));
    m_allowed_planning_time_spinbox = new QDoubleSpinBox;
    m_allowed_planning_time_spinbox->setMinimum(1.0);
    m_allowed_planning_time_spinbox->setMaximum(120.0);
    m_allowed_planning_time_spinbox->setSingleStep(1.0);
    m_allowed_planning_time_spinbox->setWrapping(false);

    planner_settings_layout->addWidget(planner_name_label,              0, 0);
    planner_settings_layout->addWidget(m_planner_name_combo_box,        0, 1);
    planner_settings_layout->addWidget(planner_id_label,                1, 0);
    planner_settings_layout->addWidget(m_planner_id_combo_box,          1, 1);
    planner_settings_layout->addWidget(num_attempts_label,              2, 0);
    planner_settings_layout->addWidget(m_num_planning_attempts_spinbox, 2, 1);
    planner_settings_layout->addWidget(allowed_planning_time_label,     3, 0);
    planner_settings_layout->addWidget(m_allowed_planning_time_spinbox, 3, 1);

    planner_settings_group->setLayout(planner_settings_layout);
    return planner_settings_group;
}

QGroupBox* MoveGroupCommandPanel::setupGoalConstraintsGroup()
{
    QGroupBox* goal_constraints_group = new QGroupBox(tr("Goal Constraints"));
    QGridLayout* goal_constraints_layout = new QGridLayout;

    // Tolerance Group
    QGroupBox* tolerance_group = new QGroupBox(tr("Tolerances"));
    QGridLayout* tolerance_layout = new QGridLayout;

    m_joint_tol_spinbox = new QDoubleSpinBox;
    m_joint_tol_spinbox->setMinimum(-180.0);
    m_joint_tol_spinbox->setMaximum( 180.0);
    m_joint_tol_spinbox->setSingleStep(1.0);
    m_joint_tol_spinbox->setWrapping(false);

    m_pos_tol_spinbox = new QDoubleSpinBox;
    m_pos_tol_spinbox->setMinimum(-1.0);
    m_pos_tol_spinbox->setMaximum( 1.0);
    m_pos_tol_spinbox->setSingleStep(0.01);
    m_pos_tol_spinbox->setWrapping(false);

    m_rot_tol_spinbox = new QDoubleSpinBox;
    m_rot_tol_spinbox->setMinimum(0.0);
    m_rot_tol_spinbox->setMaximum(180.0);
    m_rot_tol_spinbox->setSingleStep(1.0);
    m_rot_tol_spinbox->setWrapping(false);

    tolerance_layout->addWidget(new QLabel(tr("Position (m)")), 0, 0);
    tolerance_layout->addWidget(m_pos_tol_spinbox, 0, 1);
    tolerance_layout->addWidget(new QLabel(tr("Orientation (deg)")), 1, 0);
    tolerance_layout->addWidget(m_rot_tol_spinbox, 1, 1);
    tolerance_layout->addWidget(new QLabel(tr("Joint (deg)")), 2, 0);
    tolerance_layout->addWidget(m_joint_tol_spinbox, 2, 1);

    tolerance_group->setLayout(tolerance_layout);
    // End Tolerance Group

    // Workspace Group
    QGroupBox* workspace_group = new QGroupBox(tr("Workspace Parameters"));
    QGridLayout* workspace_layout = new QGridLayout;

    m_workspace_frame_combo_box = new QComboBox;

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
    // End Workspace Group

    // Goal Command Group
    QGroupBox* command_group = new QGroupBox(tr("Position Command"));
    QVBoxLayout* command_layout = new QVBoxLayout;
    m_var_cmd_widget = new JointVariableCommandWidget(m_model.getRobotCommandModel());
    command_layout->addWidget(m_var_cmd_widget);
    command_group->setLayout(command_layout);
    // End Goal Command Group

    goal_constraints_layout->addWidget(tolerance_group, 0, 0, 1, 2);
    goal_constraints_layout->addWidget(workspace_group, 1, 0, 1, 2);
    goal_constraints_layout->addWidget(command_group, 2, 0, 1, 2);

    goal_constraints_group->setLayout(goal_constraints_layout);
    return goal_constraints_group;
}

QGroupBox* MoveGroupCommandPanel::setupCommandsGroup()
{
    QGroupBox* commands_group_box = new QGroupBox(tr("Commands"));
    QVBoxLayout* commands_group_layout = new QVBoxLayout;

    m_plan_to_position_button = new QPushButton(tr("Plan to Position"));
    m_move_to_position_button = new QPushButton(tr("Move to Position"));
    m_plan_to_configuration_button = new QPushButton(tr("Plan To Configuration"));
    m_move_to_configuration_button = new QPushButton(tr("Move To Configuration"));
    m_copy_current_state_button = new QPushButton(tr("Copy Current State"));

    commands_group_layout->addWidget(m_plan_to_position_button);
    commands_group_layout->addWidget(m_move_to_position_button);
    commands_group_layout->addWidget(m_plan_to_configuration_button);
    commands_group_layout->addWidget(m_move_to_configuration_button);
    commands_group_layout->addWidget(m_copy_current_state_button);

    commands_group_box->setLayout(commands_group_layout);
    return commands_group_box;
}

void MoveGroupCommandPanel::syncPlannerNameComboBox()
{
    assert(m_planner_name_combo_box);
    // set the selected item to match the current planner
    const std::string planner_name = m_model.plannerName();
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
    assert(m_planner_id_combo_box);
    // set the selected item to match the current planner id
    const std::string planner_id = m_model.plannerID();
    for (int i = 0; i < m_planner_id_combo_box->count(); ++i) {
        if (m_planner_id_combo_box->itemText(i).toStdString() == planner_id) {
            m_planner_id_combo_box->setCurrentIndex(i);
            break;
        }
    }
}

void MoveGroupCommandPanel::syncNumPlanningAttemptsSpinBox()
{
    assert(m_num_planning_attempts_spinbox);
    m_num_planning_attempts_spinbox->setValue(m_model.numPlanningAttempts());
}

void MoveGroupCommandPanel::syncAllowedPlanningTimeSpinBox()
{
    assert(m_allowed_planning_time_spinbox);
    m_allowed_planning_time_spinbox->setValue(m_model.allowedPlanningTime());
}

void MoveGroupCommandPanel::syncGoalPositionToleranceSpinBox()
{
    assert(m_pos_tol_spinbox);
    m_pos_tol_spinbox->setValue(m_model.goalPositionTolerance());
}

void MoveGroupCommandPanel::syncGoalOrientationToleranceSpinBox()
{
    assert(m_rot_tol_spinbox);
    m_rot_tol_spinbox->setValue(m_model.goalOrientationTolerance());
}

void MoveGroupCommandPanel::syncGoalJointToleranceSpinBox()
{
    assert(m_joint_tol_spinbox);
    m_joint_tol_spinbox->setValue(m_model.goalJointTolerance());
}

void MoveGroupCommandPanel::syncWorkspaceWidgets()
{
    assert(m_workspace_min_x_spinbox && m_workspace_max_x_spinbox &&
            m_workspace_min_y_spinbox && m_workspace_max_y_spinbox &&
            m_workspace_min_z_spinbox && m_workspace_max_z_spinbox);
    bool found = false;
    for (int i = 0; i < m_workspace_frame_combo_box->count(); ++i) {
        if (m_workspace_frame_combo_box->itemText(i).toStdString() ==
                m_model.workspace().header.frame_id)
        {
            m_workspace_frame_combo_box->setCurrentIndex(i);
            found = true;
            break;
        }
    }
    if (!found) {
        m_workspace_frame_combo_box->setCurrentIndex(-1);
    }
    m_workspace_min_x_spinbox->setValue(m_model.workspace().min_corner.x);
    m_workspace_min_y_spinbox->setValue(m_model.workspace().min_corner.y);
    m_workspace_min_z_spinbox->setValue(m_model.workspace().min_corner.z);
    m_workspace_max_x_spinbox->setValue(m_model.workspace().max_corner.x);
    m_workspace_max_y_spinbox->setValue(m_model.workspace().max_corner.y);
    m_workspace_max_z_spinbox->setValue(m_model.workspace().max_corner.z);

    m_marker_pub.publish(getWorkspaceVisualization());
}

void MoveGroupCommandPanel::updateRobotVisualization()
{
    ROS_DEBUG_NAMED(LOG, "Update robot visualization");

    auto robot_model = m_model.robotModel();
    auto* robot_state = m_model.robotState();

    assert(robot_model && "Robot Model must be loaded before visualization");
    assert(robot_state != NULL && "Robot State must be initialized before visualization");

    const bool collision_markers = true;
    const bool include_attached = true;
    visualization_msgs::MarkerArray ma;
    if (collision_markers) {
         getRobotCollisionMarkers(
                 ma, *robot_state, robot_model->getLinkModelNames(), include_attached);
    }
    else {
        robot_state->getRobotMarkers(
                ma, robot_model->getLinkModelNames(), include_attached);
    }

    const std::string ns = robot_model->getName() + std::string("_command");
    int id = 0;
    for (auto& marker : ma.markers) {
        marker.mesh_use_embedded_materials = false;

        float r_base = 0.4f; // (float)100 / (float)255;
        float g_base = 0.4f; // (float)159 / (float)255;
        float b_base = 0.4f; // (float)237 / (float)255;
        boost::tribool valid = m_model.robotStateValidity();
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

    int cid = 0;
    for (auto& contact : m_model.contacts()) {
        const double arrow_len = 1.0;
        const Eigen::Vector3d arrow_pos =
                Eigen::Vector3d(
                        contact.position.x,
                        contact.position.y,
                        contact.position.z) +
                Eigen::Vector3d(0, 0, arrow_len);
        const Eigen::Affine3d arrow_transform(
                Eigen::Translation3d(arrow_pos) *
                Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

        visualization_msgs::Marker m;
        m.header.frame_id = robot_model->getModelFrame();
        m.ns = "contacts";
        m.id = cid++;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        tf::poseEigenToMsg(arrow_transform, m.pose);
        m.scale.x = arrow_len;
        m.scale.y = m.scale.z = 0.02;
        m.color.r = 1.0f;
        m.color.g = 0.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0);
        ma.markers.push_back(m);
    }

    m_marker_pub.publish(ma);
}

void MoveGroupCommandPanel::planToGoalPose()
{
    if (!m_model.planToGoalPose()) {
        ROS_ERROR("This should be a message box");
    }
}

void MoveGroupCommandPanel::moveToGoalPose()
{
    if (!m_model.moveToGoalPose()) {
        ROS_ERROR("This should also be a message box");
    }
}

void MoveGroupCommandPanel::planToGoalConfiguration()
{
    if (!m_model.planToGoalConfiguration()) {
        ROS_ERROR("This should be a message box");
    }
}

void MoveGroupCommandPanel::moveToGoalConfiguration()
{
    if (!m_model.moveToGoalConfiguration()) {
        ROS_ERROR("This should be a message box");
    }
}

void MoveGroupCommandPanel::copyCurrentState()
{
    m_model.copyCurrentState();
}

void MoveGroupCommandPanel::setGoalJointTolerance(double tol_deg)
{
    m_model.setGoalJointTolerance(tol_deg);
}

void MoveGroupCommandPanel::setGoalPositionTolerance(double tol_m)
{
    m_model.setGoalPositionTolerance(tol_m);
}

void MoveGroupCommandPanel::setGoalOrientationTolerance(double tol_deg)
{
    m_model.setGoalOrientationTolerance(tol_deg);
}

void MoveGroupCommandPanel::setCurrentPlanner(const QString& name)
{
    m_model.setPlannerName(name.toStdString());
}

void MoveGroupCommandPanel::setCurrentPlannerID(const QString& id)
{
    m_model.setPlannerID(id.toStdString());
}

void MoveGroupCommandPanel::setWorkspaceFrame(const QString& frame)
{
    moveit_msgs::WorkspaceParameters params = m_model.workspace();
    params.header.frame_id = frame.toStdString();
    m_model.setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMinX(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model.workspace();
    params.min_corner.x = value;
    m_model.setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMinY(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model.workspace();
    params.min_corner.y = value;
    m_model.setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMinZ(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model.workspace();
    params.min_corner.z = value;
    m_model.setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMaxX(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model.workspace();
    params.max_corner.x = value;
    m_model.setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMaxY(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model.workspace();
    params.max_corner.y = value;
    m_model.setWorkspace(params);
}

void MoveGroupCommandPanel::setWorkspaceMaxZ(double value)
{
    moveit_msgs::WorkspaceParameters params = m_model.workspace();
    params.max_corner.z = value;
    m_model.setWorkspace(params);
}

visualization_msgs::MarkerArray
MoveGroupCommandPanel::getWorkspaceVisualization() const
{
    // shamefully copypasta'd from sbpl_manipulation and leatherman

    visualization_msgs::MarkerArray ma;

    const auto& workspace = m_model.workspace();

    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = workspace.header.frame_id;
    marker.ns = "workspace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration(0);

    std::vector<geometry_msgs::Point> pts(10);
    const double origin_x = workspace.min_corner.x;
    const double origin_y = workspace.min_corner.y;
    const double origin_z = workspace.min_corner.z;
    const double dim_x = workspace.max_corner.x - workspace.min_corner.x;
    const double dim_y = workspace.max_corner.y - workspace.min_corner.y;
    const double dim_z = workspace.max_corner.z - workspace.min_corner.z;
    pts[0].x = origin_x;        pts[0].y = origin_y;        pts[0].z = origin_z;
    pts[1].x = origin_x+dim_x;  pts[1].y = origin_y;        pts[1].z = origin_z;
    pts[2].x = origin_x+dim_x;  pts[2].y = origin_y+dim_y;  pts[2].z = origin_z;
    pts[3].x = origin_x;        pts[3].y = origin_y+dim_y;  pts[3].z = origin_z;
    pts[4].x = origin_x;        pts[4].y = origin_y;        pts[4].z = origin_z;
    pts[5].x = origin_x;        pts[5].y = origin_y;        pts[5].z = origin_z+dim_z;
    pts[6].x = origin_x+dim_x;  pts[6].y = origin_y;        pts[6].z = origin_z+dim_z;
    pts[7].x = origin_x+dim_x;  pts[7].y = origin_y+dim_y;  pts[7].z = origin_z+dim_z;
    pts[8].x = origin_x;        pts[8].y = origin_y+dim_y;  pts[8].z = origin_z+dim_z;
    pts[9].x = origin_x;        pts[9].y = origin_y;        pts[9].z = origin_z+dim_z;
    marker.points = std::move(pts);

    ma.markers.push_back(std::move(marker));
    return ma;
}

void MoveGroupCommandPanel::getRobotCollisionMarkers(
    visualization_msgs::MarkerArray& ma,
    const moveit::core::RobotState& state,
    const std::vector<std::string>& link_names,
    bool include_attached) const
{
    auto urdf = state.getRobotModel()->getURDF();

    ros::Time tm = ros::Time::now();
    for (std::size_t i = 0; i < link_names.size(); ++i) {
        ROS_DEBUG("Trying to get marker for link '%s'", link_names[i].c_str());
        const moveit::core::LinkModel* lm = state.getRobotModel()->getLinkModel(link_names[i]);
        auto urdf_link = urdf->getLink(link_names[i]);

        if (!lm || !urdf_link) {
            continue;
        }

        if (include_attached) {
            std::vector<const moveit::core::AttachedBody*> attached_bodies;
            state.getAttachedBodies(attached_bodies);
            for (const moveit::core::AttachedBody* ab : attached_bodies) {
                if (ab->getAttachedLink() == lm) {
                    for (std::size_t j = 0 ; j < ab->getShapes().size() ; ++j) {
                        visualization_msgs::Marker att_mark;
                        att_mark.header.frame_id = state.getRobotModel()->getModelFrame();
                        att_mark.header.stamp = tm;
                        if (shapes::constructMarkerFromShape(ab->getShapes()[j].get(), att_mark)) {
                            // if the object is invisible (0 volume) we skip it
                            if (fabs(att_mark.scale.x * att_mark.scale.y * att_mark.scale.z) <
                                    std::numeric_limits<float>::epsilon())
                            {
                                continue;
                            }
                            tf::poseEigenToMsg(ab->getGlobalCollisionBodyTransforms()[j], att_mark.pose);
                            ma.markers.push_back(att_mark);
                        }
                    }
                }
            }
        }

        if (lm->getShapes().empty()) {
            continue;
        }

        for (std::size_t j = 0 ; j < lm->getShapes().size() ; ++j) {
            auto shape = lm->getShapes()[j];
            // we're going to roll our own markers for meshes later
            if (shape->type == shapes::ShapeType::MESH) {
                continue;
            }

            visualization_msgs::Marker m;
            m.header.frame_id = state.getRobotModel()->getModelFrame();
            m.header.stamp = tm;

            // hope this does a good job for non-meshes
            if (!shapes::constructMarkerFromShape(shape.get(), m)) {
                continue;
            }

            // if the object is invisible (0 volume) we skip it
            if (fabs(m.scale.x * m.scale.y * m.scale.z) <
                    std::numeric_limits<float>::epsilon())
            {
                continue;
            }

            const auto& T_model_shape = state.getCollisionBodyTransform(lm, j);
            tf::poseEigenToMsg(T_model_shape, m.pose);

            ma.markers.push_back(m);
        }

        // roll our own markers for mesh shapes

        // gather all urdf collision elements (take care of some crufty stuff here)
        std::vector<boost::shared_ptr<urdf::Collision>> collisions;
        if (urdf_link->collision) {
            collisions.push_back(urdf_link->collision);
        }
        else {
            collisions.assign(urdf_link->collision_array.begin(), urdf_link->collision_array.end());
        }

        size_t cidx = 0;
        for (const auto& collision : collisions) {
            if (collision->geometry->type == urdf::Geometry::MESH) {
                const urdf::Mesh* mesh = (const urdf::Mesh*)collision->geometry.get();
                visualization_msgs::Marker m;
                m.header.frame_id = state.getRobotModel()->getModelFrame();
                m.header.stamp = tm;
                m.type = m.MESH_RESOURCE;
                m.mesh_use_embedded_materials = true;
                m.mesh_resource = mesh->filename;
                m.scale.x = mesh->scale.x;
                m.scale.y = mesh->scale.y;
                m.scale.z = mesh->scale.z;

                // Aha! lucky guess
                const auto& T_model_shape = state.getCollisionBodyTransform(lm, cidx);
                tf::poseEigenToMsg(T_model_shape, m.pose);

                ma.markers.push_back(m);
            }
            ++cidx;
        }
    }
}

QVBoxLayout* MoveGroupCommandPanel::mainLayout()
{
    QVBoxLayout* parent_layout = qobject_cast<QVBoxLayout*>(layout());
    QScrollArea* scroll_area = qobject_cast<QScrollArea*>(parent_layout->itemAt(0)->widget());
    QVBoxLayout* main_layout = qobject_cast<QVBoxLayout*>(scroll_area->widget()->layout());
    return main_layout;
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbpl_interface::MoveGroupCommandPanel, rviz::Panel)
