#include "joint_variable_command_widget.h"

// system includes
#include <Eigen/Dense>
#include <ros/console.h>
#include <sbpl_geometry_utils/utils.h>
#include <leatherman/print.h>

// module includes
#include "move_group_command_model.h"

namespace sbpl_interface {

JointVariableCommandWidget::JointVariableCommandWidget(
    MoveGroupCommandModel* model,
    QWidget* parent)
:
    Base(parent),
    m_model(model),
    m_ignore_sync(false)
{
    connect(model, SIGNAL(robotLoaded()), this, SLOT(updateRobotControls()));

    QGridLayout* glayout = new QGridLayout;

    m_joint_groups_combo_box = new QComboBox;

    glayout->addWidget(new QLabel(tr("Joint Group:")), 0, 0);
    glayout->addWidget(m_joint_groups_combo_box, 0, 1);
    setLayout(glayout);

    connect(m_joint_groups_combo_box,
            SIGNAL(currentIndexChanged(const QString&)),
            this,
            SLOT(setJointGroup(const QString&)));
}

JointVariableCommandWidget::~JointVariableCommandWidget()
{
    // TODO: are the spinboxes/labels that are not currently attached to the
    // widget deleted when this entire widget is deleted? Probably not, and in
    // that case we may have to delete them manually
}

void JointVariableCommandWidget::displayJointGroupCommands(
    const std::string& group_name)
{
    ROS_INFO("display joint group commands");
    QGridLayout* glayout = qobject_cast<QGridLayout*>(layout());
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
            robot_model->getJointModelGroup(group_name);
    if (!jmg) {
        ROS_ERROR("Joint Group '%s' does not exist within Robot Model '%s'", group_name.c_str(), robot_model->getName().c_str());
        return;
    }

    ROS_INFO("remove everything from layout");

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

    // add the master label and group combo box back in
    glayout->addWidget(new QLabel(tr("Joint Group:")), 0, 0);
    glayout->addWidget(m_joint_groups_combo_box, 0, 1);
    m_joint_groups_combo_box->show(); // this gets hidden above

    ROS_INFO("add joint group controls to layout");

    // gather (label, spinbox) pairs to be visible in the layout
    // TODO: this could probably be made more better since spinboxes are
    // constructed per-joint and should be able to be looked up from the joint
    // they derive from
    std::vector<std::pair<QLabel*, QDoubleSpinBox*>> group_widgets;
    const auto& joint_models = jmg->getActiveJointModels();
    for (const moveit::core::JointModel* joint_model : joint_models) {
        int bidx = joint_model->getFirstVariableIndex();
        for (int vidx = bidx; vidx < bidx + joint_model->getVariableCount(); ++vidx) {
            const std::string& var_name = robot_model->getVariableNames()[vidx];
            assert(vidx >= 0 && vidx < m_vind_to_spinbox.size());
            for (QDoubleSpinBox* spinbox : m_vind_to_spinbox[vidx]) {
                // find the label corresponding to this spinbox
                auto it = std::find(m_spinboxes.begin(), m_spinboxes.end(), spinbox);
                auto sidx = std::distance(m_spinboxes.begin(), it);
                group_widgets.emplace_back(m_labels[sidx], spinbox);
            }
        }
    }

    // erase duplicate entries...see above for insight to why this is needed
    auto uit = std::unique(group_widgets.begin(), group_widgets.end());
    group_widgets.erase(uit, group_widgets.end());

    // add (label, spinbox) widgets to the layout
    int row = glayout->rowCount();
    for (auto& entry : group_widgets) {
        ROS_INFO("add spinbox %p and label %p to layout", entry.first, entry.second);
        glayout->addWidget(entry.first, row, 0);
        glayout->addWidget(entry.second, row, 1);
        entry.first->show();
        entry.second->show();
        ++row;
    }

    ROS_INFO("Layout contains %d controls", glayout->count());

    ///////////////////////////////////////////////////////////////////////
    // KEEP THESE CALLS BECAUSE THEY USED TO BE THE ONLY WAY TO AUTOMATIC
    // RESIZING WORK AND THEY HAVEN'T BROKEN YET

//    for (int i = 0; i < glayout->count(); ++i) {
//        glayout->itemAt(i)->invalidate();
//    }
//
//    glayout->invalidate();
    layout()->invalidate();
//    glayout->activate();
//    layout()->activate();
//    glayout->update();
    layout()->update();

    //////////////////////////////////////////////////////////////////////
}

/// \brief Set the values of all spinboxes to match those in the robot state
void JointVariableCommandWidget::syncSpinBoxes()
{
    if (m_ignore_sync) {
        return;
    }

    ROS_INFO("sync spinboxes");
    if (!m_model->isRobotLoaded()) {
        ROS_WARN("Robot not yet loaded");
        return;
    }

    auto robot_model = m_model->robotModel();
    auto robot_state = m_model->robotState();

    // ugh this needs to be batched
    const auto& active_joints = robot_model->getActiveJointModels();
    for (const moveit::core::JointModel* jm : active_joints) {
        if (jm->getType() == moveit::core::JointModel::FLOATING) {
            // only floating joints have many-to-many joint variable to
            // spinbox mappings
            std::vector<QDoubleSpinBox*> qspinboxes;
            // set the values of the translation variable spinboxes and gather
            // the rotation quaternion values
            double qvars[4];
            int bidx = jm->getFirstVariableIndex();
            for (int vi = bidx; vi < bidx + jm->getVariableCount(); ++vi) {
                const int lvi = vi - jm->getFirstVariableIndex();
                const std::string& var_name = jm->getLocalVariableNames()[lvi];
                if (var_name == "trans_x" ||
                    var_name == "trans_y" ||
                    var_name == "trans_z")
                {
                    assert(m_vind_to_spinbox[vi].size() == 1);
                    QDoubleSpinBox* spinbox = m_vind_to_spinbox[vi][0];
                    double value = robot_state->getVariablePosition(vi);
                    if (value != spinbox->value()) {
                        spinbox->setValue(value);
                    }
                } else if (var_name == "rot_w") {
                    assert(m_vind_to_spinbox[vi].size() == 3);
                    qspinboxes = m_vind_to_spinbox[vi];
                    qvars[0] = robot_state->getVariablePosition(vi);
                } else if (var_name == "rot_x") {
                    qvars[1] = robot_state->getVariablePosition(vi);
                } else if (var_name == "rot_y") {
                    qvars[2] = robot_state->getVariablePosition(vi);
                } else if (var_name == "rot_z") {
                    qvars[3] = robot_state->getVariablePosition(vi);
                }
            }

            ROS_INFO("sync rpy from quaternion (%0.3f, %0.3f, %0.3f, %0.3f)", qvars[0], qvars[1], qvars[2], qvars[3]);
            // simultaneously set the values for the rpy spinboxes
            Eigen::Affine3d rot(Eigen::Quaterniond(
                    qvars[0], qvars[1], qvars[2], qvars[3]));
            Eigen::Vector3d ypr = rot.rotation().eulerAngles(2, 1, 0);

            Eigen::Affine3d A(
                Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX()));

            double are_diff = A.isApprox(rot, 0.001);

            // round to the nearest degree to try and get more stable results
            double vY = std::round(sbpl::utils::ToDegrees(ypr[0]));
            double vP = std::round(sbpl::utils::ToDegrees(ypr[1]));
            double vR = std::round(sbpl::utils::ToDegrees(ypr[2]));
            // break the cycle
            if ((qspinboxes[0]->value() != vR ||
                qspinboxes[1]->value() != vP ||
                qspinboxes[2]->value() != vY) && are_diff)
            {
                disconnect(qspinboxes[0], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                disconnect(qspinboxes[1], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                disconnect(qspinboxes[2], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                ROS_INFO("sync rpy spinboxes with values (%0.3f, %0.3f, %0.3f)", vR, vP, vY);
                qspinboxes[0]->setValue(vR);
                qspinboxes[1]->setValue(vP);
                qspinboxes[2]->setValue(vY);
                connect(qspinboxes[0], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                connect(qspinboxes[1], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                connect(qspinboxes[2], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
            }
        } else {
            int bidx = jm->getFirstVariableIndex();
            for (int vi = bidx; vi < bidx + jm->getVariableCount(); ++vi) {
                // all others should have a 1-1 mapping
                assert(m_vind_to_spinbox[vi].size() == 1);
                QDoubleSpinBox* spinbox = m_vind_to_spinbox[vi][0];
                if (isVariableAngle(vi)) {
                    double value = sbpl::utils::ToDegrees(
                            robot_state->getVariablePosition(vi));
                    if (value != spinbox->value()) {
                        spinbox->setValue(value);
                    }
                } else {
                    double value = robot_state->getVariablePosition(vi);
                    // this check is required because the internal value of
                    // the spinbox may differ from the displayed value.
                    // Apparently, scrolling the spinbox by a step less than
                    // the precision will update the internal value, but
                    // calling setValue will ensure that the internal value
                    // is the same as the value displayed. The absence of
                    // this check can result in not being able to update a
                    // joint variable
                    if (value != spinbox->value()) {
                        spinbox->setValue(value);
                    }
                }
            }
        }
    }
}

/// \brief Set the choice in the group combo box to match the active group
void JointVariableCommandWidget::syncPlanningJointGroupComboBox()
{
    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();
    if (!robot_model) {
        // TODO: set to empty selection?...yeah probably
        return;
    }

    assert(m_joint_groups_combo_box);

    // set the selected item to match the active planning joint group
    if (!robot_model->getJointModelGroups().empty()) {
        const int ajg_idx = m_joint_groups_combo_box->findText(
                QString::fromStdString(m_model->planningJointGroupName()));
        m_joint_groups_combo_box->setCurrentIndex(ajg_idx);
    }
}

/// \brief Update the spinboxes and group selections for a new robot
void JointVariableCommandWidget::updateRobotControls()
{
    ROS_INFO("update robot controls");
    QGridLayout* grid_layout = qobject_cast<QGridLayout*>(layout());
    if (!grid_layout) {
        ROS_ERROR("JointVariableCommandWidget layout is not a QGridLayout");
        return;
    }

    auto robot_model = m_model->robotModel();
    if (!robot_model) {
        ROS_ERROR("cannot update robot controls from null robot model");
        return;
    }

    const size_t var_count = robot_model->getVariableCount();

    m_spinbox_to_vind.clear();
    m_vind_to_spinbox.resize(var_count);
    m_spinboxes.clear(); // TODO: delete old spinboxes?
    m_labels.clear();

    const auto& active_joints = robot_model->getActiveJointModels();
    for (const moveit::core::JointModel* jm : active_joints) {
        switch (jm->getType()) {
        case moveit::core::JointModel::FLOATING: {
            QDoubleSpinBox* trans_x_spinbox = createRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/trans_x"));
            QDoubleSpinBox* trans_y_spinbox = createRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/trans_y"));
            QDoubleSpinBox* trans_z_spinbox = createRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/trans_y"));
            QDoubleSpinBox* rot_R_spinbox = createRollVariableSpinBox();
            QDoubleSpinBox* rot_P_spinbox = createPitchVariableSpinBox();
            QDoubleSpinBox* rot_Y_spinbox = createYawVariableSpinBox();

            m_spinboxes.push_back(trans_x_spinbox);
            m_spinboxes.push_back(trans_y_spinbox);
            m_spinboxes.push_back(trans_z_spinbox);
            m_spinboxes.push_back(rot_R_spinbox);
            m_spinboxes.push_back(rot_P_spinbox);
            m_spinboxes.push_back(rot_Y_spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/trans_x")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/trans_y")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/trans_z")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/rot_R")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/rot_P")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/rot_Y")));

            m_spinbox_to_vind[trans_x_spinbox] = std::vector<int>(1);
            m_spinbox_to_vind[trans_y_spinbox] = std::vector<int>(1);
            m_spinbox_to_vind[trans_z_spinbox] = std::vector<int>(1);

            // indices for variables (qw, qx, qy, qz) of this joint
            m_spinbox_to_vind[rot_R_spinbox] = std::vector<int>(4);
            m_spinbox_to_vind[rot_P_spinbox] = std::vector<int>(4);
            m_spinbox_to_vind[rot_Y_spinbox] = std::vector<int>(4);

            // map joint variable indices to spinboxes and vice versa
            for (int vidx = jm->getFirstVariableIndex();
                vidx < jm->getFirstVariableIndex() + jm->getVariableCount();
                ++vidx)
            {
                int lvidx = vidx - jm->getFirstVariableIndex();
                // check which variable it is (makes no assumption about the
                // order of variables within the joint) but does make
                // assumptions about the names of the variables ...expects
                // standard moveit naming conventions for floating joint
                // variables
                const std::string& var_name = jm->getLocalVariableNames()[lvidx];
                if (var_name == "trans_x") {
                    m_vind_to_spinbox[vidx] = { trans_x_spinbox };
                    m_spinbox_to_vind[trans_x_spinbox][0] = vidx;
                } else if (var_name == "trans_y") {
                    m_vind_to_spinbox[vidx] = { trans_y_spinbox };
                    m_spinbox_to_vind[trans_y_spinbox][0] = vidx;
                } else if (var_name == "trans_z") {
                    m_vind_to_spinbox[vidx] = { trans_z_spinbox };
                    m_spinbox_to_vind[trans_z_spinbox][0] = vidx;
                } else if (var_name == "rot_w") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][0] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][0] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][0] = vidx;
                } else if (var_name == "rot_x") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][1] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][1] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][1] = vidx;
                } else if (var_name == "rot_y") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][2] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][2] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][2] = vidx;
                } else if (var_name == "rot_z") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][3] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][3] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][3] = vidx;
                } else {
                    ROS_ERROR("unrecognized local joint variable name");
                }
            }
        }   break;
        case moveit::core::JointModel::PLANAR: {
            QDoubleSpinBox* x_spinbox = createRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/x"));
            QDoubleSpinBox* y_spinbox = createRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/y"));
            QDoubleSpinBox* theta_spinbox = createAngleVariableSpinBox();

            m_spinboxes.push_back(x_spinbox);
            m_spinboxes.push_back(y_spinbox);
            m_spinboxes.push_back(theta_spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/x")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/y")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/theta")));

            for (int vidx = jm->getFirstVariableIndex();
                vidx < jm->getFirstVariableIndex() + jm->getVariableCount();
                ++vidx)
            {
                int lvidx = vidx - jm->getFirstVariableIndex();
                const std::string& var_name = jm->getLocalVariableNames()[lvidx];
                if (var_name == "x") {
                    m_vind_to_spinbox[vidx] = { x_spinbox };
                    m_spinbox_to_vind[x_spinbox] = { vidx} ;
                } else if (var_name == "y") {
                    m_vind_to_spinbox[vidx] = { y_spinbox };
                    m_spinbox_to_vind[y_spinbox] = { vidx };
                } else if (var_name == "theta") {
                    m_vind_to_spinbox[vidx] = { theta_spinbox };
                    m_spinbox_to_vind[theta_spinbox] = { vidx };
                } else {
                    ROS_ERROR("unrecognized local joint variable name");
                }
            }
        }   break;
        case moveit::core::JointModel::PRISMATIC: {
            QDoubleSpinBox* spinbox = createRealVariableSpinBox(jm->getVariableBounds()[0]);
            m_spinboxes.push_back(spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName())));
            int vidx = jm->getFirstVariableIndex();
            m_vind_to_spinbox[vidx] = { spinbox };
            m_spinbox_to_vind[spinbox] = { vidx };
        }   break;
        case moveit::core::JointModel::REVOLUTE: {
            QDoubleSpinBox* spinbox = createRevoluteVariableSpinBox(jm->getVariableBounds()[0]);
            m_spinboxes.push_back(spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName())));
            int vidx = jm->getFirstVariableIndex();
            m_vind_to_spinbox[vidx] = { spinbox };
            m_spinbox_to_vind[spinbox] = { vidx };
        }   break;
        case moveit::core::JointModel::FIXED:
        case moveit::core::JointModel::UNKNOWN: {
            ROS_ERROR("invalid joint type %d found in active joint models", (int)jm->getType());
        }   break;
        }
    }

    // NOTE: update the combo box after the variable spinboxes have been created
    // since setJointGroup will be called by signals emitted by the combo box
    m_joint_groups_combo_box->clear();

    const auto& group_names = robot_model->getJointModelGroupNames();
    for (const std::string& group_name : group_names) {
        m_joint_groups_combo_box->addItem(QString::fromStdString(group_name));
    }

    // set the combo box to the value of the active planning group
    if (!robot_model->getJointModelGroups().empty()) {
        const int ajg_idx = m_joint_groups_combo_box->findText(
                QString::fromStdString(m_model->planningJointGroupName()));
        m_joint_groups_combo_box->setCurrentIndex(ajg_idx);
    }

    // connect all spinboxes to slot for setting joint variables
    for (QDoubleSpinBox* spinbox : m_spinboxes) {
        connect(spinbox, SIGNAL(valueChanged(double)),
                this, SLOT(setJointVariableFromSpinBox(double)));
    }
}

QDoubleSpinBox* JointVariableCommandWidget::createRealVariableSpinBox(
    const moveit::core::VariableBounds& bounds)
{
    ROS_INFO("create real variable spinbox");
    QDoubleSpinBox* spinbox = new QDoubleSpinBox;
    double min, max, step;
    if (bounds.position_bounded_) {
        // why are floating joint linear variables bounded with infinite bounds?
        if (bounds.min_position_ == -std::numeric_limits<double>::infinity()) {
            min = -100.0;
        } else {
            min = bounds.min_position_;
        }
        if (bounds.max_position_ == std::numeric_limits<double>::infinity()) {
            max = 100.0;
        } else {
            max = bounds.max_position_;
        }

        if (bounds.max_position_ == std::numeric_limits<double>::infinity() ||
            bounds.min_position_ == -std::numeric_limits<double>::infinity())
        {
            step = 0.01;
        } else {
            step = (max - min) / 100.0;
        }
    } else {
        min = -100.0;
        max = 100.0;
        step = 0.01;
    }

    spinbox->setMinimum(min);
    spinbox->setMaximum(max);
    ROS_INFO(" -> single step for real variable: %f", step);

    // TODO: compute the number of decimals required to display a
    // change in the joint variable or round the step up to the
    // nearest number of decimals desired to be displayed
    spinbox->setDecimals(3);
    spinbox->setSingleStep(step);
    spinbox->setWrapping(false);
    return spinbox;
}

QDoubleSpinBox* JointVariableCommandWidget::createRollVariableSpinBox()
{
    ROS_INFO("create roll variable spinbox");
    // TODO: limit bounds
    QDoubleSpinBox* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(-180.0);
    spinbox->setMaximum(180.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    return spinbox;
}

QDoubleSpinBox* JointVariableCommandWidget::createPitchVariableSpinBox()
{
    ROS_INFO("create pitch variable spinbox");
    // TODO: limit bounds
    QDoubleSpinBox* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(-180.0);
    spinbox->setMaximum(180.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    return spinbox;
}

QDoubleSpinBox* JointVariableCommandWidget::createYawVariableSpinBox()
{
    ROS_INFO("create yaw variable spinbox");
    // TODO: limit bounds
    QDoubleSpinBox* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(0.0);
    spinbox->setMaximum(180.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    return spinbox;
}

QDoubleSpinBox* JointVariableCommandWidget::createAngleVariableSpinBox()
{
    ROS_INFO("create angle variable spinbox");
    QDoubleSpinBox* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(0.0);
    spinbox->setMaximum(359.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    return spinbox;
}

QDoubleSpinBox* JointVariableCommandWidget::createRevoluteVariableSpinBox(
    const moveit::core::VariableBounds& bounds)
{
    if (bounds.position_bounded_) {
        QDoubleSpinBox* spinbox = new QDoubleSpinBox;
        spinbox->setMinimum(
                sbpl::utils::ToDegrees(bounds.min_position_));
        spinbox->setMaximum(
                sbpl::utils::ToDegrees(bounds.max_position_));
        spinbox->setSingleStep(1.0);
        spinbox->setWrapping(false);
        return spinbox;
    } else {
        return createAngleVariableSpinBox();
    }
}

/// \brief Test if a variable is a single-dof angular variable
///
/// Angle variables are treated differently by displaying their values in
/// degrees while the underlying value is expressed in radians
bool JointVariableCommandWidget::isVariableAngle(int vidx) const
{
    auto robot_model = m_model->robotModel();
    if (!robot_model) {
        ROS_WARN("Asking whether variable %d in uninitialized robot is an angle", vidx);
        return false;
    }

    const moveit::core::JointModel* jm = robot_model->getJointOfVariable(vidx);

    const std::string& var_name = robot_model->getVariableNames()[vidx];

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

/// \brief Set the model's active group  and display controls for group variables
void JointVariableCommandWidget::setJointGroup(const QString& group_name)
{
    std::string group_name_str = group_name.toStdString();
    displayJointGroupCommands(group_name_str);
    m_model->setPlanningJointGroup(group_name_str);
}

/// \brief Update the model variable from a spinbox
void JointVariableCommandWidget::setJointVariableFromSpinBox(double value)
{
    QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(sender());
    if (!spinbox) {
        ROS_WARN("setJointVariableFromSpinBox not called from a spinbox");
        return;
    }

    assert(m_spinbox_to_vind.find(spinbox) != m_spinbox_to_vind.end());

    const std::vector<int>& variables = m_spinbox_to_vind[spinbox];

    ROS_INFO("Update joint variables %s from spinbox %p with value %0.3f", to_string(variables).c_str(), spinbox, value);

    if (variables.size() == 4) {
        // so much hackery here for quaternion controls
        // need to get the values from the other spinboxes
        const std::vector<QDoubleSpinBox*>& rpy_controls =
                m_vind_to_spinbox[variables[0]];

        ROS_INFO("rpy controls: %s", to_string(rpy_controls).c_str());

        // attempt stability
        double r = std::round(rpy_controls[0]->value());
        double p = std::round(rpy_controls[1]->value());
        double y = std::round(rpy_controls[2]->value());
        ROS_INFO("set rpy values from spinbox values (%0.3f, %0.3f, %0.3f)", r, p, y);
        Eigen::Quaterniond rot(
                Eigen::AngleAxisd(sbpl::utils::ToRadians(y), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(sbpl::utils::ToRadians(p), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(sbpl::utils::ToRadians(r), Eigen::Vector3d::UnitX()));
        ROS_INFO("  set quaternion (%0.3f, %0.3f, %0.3f, %0.3f)", rot.w(), rot.x(), rot.y(), rot.z());
        m_ignore_sync = true;
        m_model->setJointVariable(variables[0], rot.w());
        m_model->setJointVariable(variables[1], rot.x());
        m_model->setJointVariable(variables[2], rot.y());
        m_model->setJointVariable(variables[3], rot.z());
        m_ignore_sync = false;
    } else if (variables.size() == 1) {
        // everything else
        if (isVariableAngle(variables[0])) {
            // convert to radians and assign
            m_model->setJointVariable(variables[0], sbpl::utils::ToRadians(value));
        } else {
            // assign without conversion
            m_model->setJointVariable(variables[0], value);
        }
    } else {
        ROS_ERROR("weird");
    }
}

} // namespace sbpl_interface
