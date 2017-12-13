#include <moveit_planners_sbpl/interface/teleop_command.h>

// system includes
#include <smpl/angles.h>

// project includes
#include <moveit_planners_sbpl/interface/robot_command_model.h>
#include <moveit_planners_sbpl/interface/utils.h>

namespace sbpl_interface {

static const char* LOG = "teleop_command";

TeleopCommand::TeleopCommand(RobotCommandModel* model)
{
    m_model = model;
    m_joy_sub = m_nh.subscribe("user_demo_joy", 5, &TeleopCommand::joyCallback, this);
    connect(m_model, SIGNAL(robotLoaded()), this, SLOT(updateRobotModel()));
    connect(m_model, SIGNAL(robotStateChanged()), this, SLOT(updateRobotState()));
}

void TeleopCommand::setActiveJointGroup(const std::string& group_name)
{
    if (m_active_group_name != group_name) {
        m_active_group_name = group_name;
        m_remote_curr_var = 0;

        ROS_DEBUG_STREAM_NAMED(LOG, "  Next joint group: " << m_active_group_name);
        auto& robot_model = m_model->getRobotModel();
        if (robot_model) {
            auto* jg = robot_model->getJointModelGroup(group_name);
            if (jg) {
                ROS_DEBUG_STREAM_NAMED(LOG, "  Active variable: " << jg->getVariableNames()[0]);
            }
        }

        Q_EMIT updateActiveJointGroup(group_name);
    }
}

void TeleopCommand::updateRobotModel()
{
    // default to the first joint group with active joint variables
    auto& robot_model = m_model->getRobotModel();
    if (m_active_group_name.empty()) {
        ROS_DEBUG_NAMED(LOG, "Choose default active joint group...");
        auto& groups = robot_model->getJointModelGroups();
        for (auto* group : groups) {
            if (group->getVariableCount() > 0) {
                ROS_DEBUG_STREAM_NAMED(LOG, "Defaulting to " <<
                        group->getVariableNames()[0] << " in group " <<
                        m_active_group_name);
                m_active_group_name = group->getName();
                m_remote_curr_var = 0;
                break;
            }
        }
    }
}

void TeleopCommand::updateRobotState()
{

}

bool ButtonPressed(
    const sensor_msgs::Joy& prev,
    const sensor_msgs::Joy& curr,
    TeleopCommand::Button button)
{
    using ButtonStateType = sensor_msgs::Joy::_buttons_type::value_type;
    return prev.buttons[(int)button] == (ButtonStateType)TeleopCommand::ButtonState::Up &&
            curr.buttons[(int)button] == (ButtonStateType)TeleopCommand::ButtonState::Down;
}

bool AxisMoved(
    const sensor_msgs::Joy& prev,
    const sensor_msgs::Joy& curr,
    TeleopCommand::Axis axis)
{
    return prev.axes[(int)axis] != curr.axes[(int)axis];
}

bool AxisNonZero(const sensor_msgs::Joy& msg, TeleopCommand::Axis axis)
{
    return msg.axes[(int)axis] != 0.0;
}

void TeleopCommand::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (!m_prev_joy) {
        ROS_DEBUG_NAMED(LOG, "Create fake previous joy message");
        auto new_joy = sensor_msgs::JoyPtr(new sensor_msgs::Joy);
        new_joy->buttons.resize((int)Button::Count);
        new_joy->axes.resize((int)Axis::Count);
        std::fill(begin(new_joy->buttons), end(new_joy->buttons), 0);
        std::fill(begin(new_joy->axes), end(new_joy->axes), 0.0);
        m_prev_joy = std::move(new_joy);
    }

    if (m_active_group_name.empty()) {
        ROS_ERROR("no active joint group");
        return;
    }

    if (m_remote_curr_var == -1) {
        ROS_ERROR("no active joints in joint group");
        return;
    }

    auto& robot_model = m_model->getRobotModel();
    if (!robot_model) {
        return;
    }

    auto* jg = robot_model->getJointModelGroup(m_active_group_name);
    if (!jg) {
        ROS_ERROR("bad! not a real joint group");
        return;
    }

    auto* robot_state = m_model->getRobotState();
    if (!robot_state) {
        return;
    }

    // A - next joint
    // X - prev joint
    // B - next joint group
    // Y - previous joint group
    if (ButtonPressed(*m_prev_joy, *msg, Button::B)) {
        cycleActiveGroup(true);
    }
    if (ButtonPressed(*m_prev_joy, *msg, Button::Y)) {
        cycleActiveGroup(false);
    }
    if (ButtonPressed(*m_prev_joy, *msg, Button::A)) {
        cycleActiveJoint(true);
    }
    if (ButtonPressed(*m_prev_joy, *msg, Button::X)) {
        cycleActiveJoint(false);
    }

    if (ButtonPressed(*m_prev_joy, *msg, Button::Back)) {
        std::vector<double> default_positions;
        jg->getVariableDefaultPositions(default_positions);
        m_model->setJointGroupPositions(jg, default_positions);
    }

    constexpr auto dt = 1.0 / 60.0;
    const auto rps = sbpl::angles::to_radians(45.0);
    const auto mps = 0.5;

    // Process trigger input for joint commands
    // TODO: detect the zero-value of the axis, since the trigger on the xbox
    // remote uses +1 to mean zero and goes down to -1 when depressed
    if (msg->axes[(int)Axis::LT] != 1.0 ||
        msg->axes[(int)Axis::RT] != 1.0
//        AxisNonZero(*msg, Axis::LT) ||
//        AxisNonZero(*msg, Axis::RT)
    )
        {
        double udp = rps * dt * msg->axes[(int)Axis::LT];
        double ddp = rps * dt * msg->axes[(int)Axis::RT];

        // get the index into the robot state of the current joint variable
        auto& var_name = jg->getVariableNames()[m_remote_curr_var];
        auto vidx = std::distance(
            begin(robot_state->getVariableNames()),
            std::find(
                begin(robot_state->getVariableNames()),
                end(robot_state->getVariableNames()),
                var_name));

        double curr_pos = robot_state->getVariablePosition(vidx);
        double next_pos = curr_pos + udp - ddp;
        ROS_DEBUG_NAMED(LOG, "set joint variable '%s' %f -> %f", var_name.c_str(), curr_pos, next_pos);

        m_model->setVariablePosition(var_name, next_pos);
    }

    // Process trigger input for ik commands
    if (AxisNonZero(*msg, Axis::RV) ||
        AxisNonZero(*msg, Axis::RH) ||
        AxisNonZero(*msg, Axis::LV) ||
        AxisNonZero(*msg, Axis::LH) ||
        AxisNonZero(*msg, Axis::DH) ||
        AxisNonZero(*msg, Axis::DV))
    {
        auto tip_links = GetTipLinkNames(*jg);
        if (!tip_links.empty()) {
            auto& tip_link = tip_links.front();

            const double dx = mps * dt * msg->axes[(int)Axis::RV];
            const double dy = mps * dt * msg->axes[(int)Axis::RH];
            const double dz = mps * dt * msg->axes[(int)Axis::LV];
            const double dY = rps * dt * msg->axes[(int)Axis::LH];
            const double dP = rps * dt * msg->axes[(int)Axis::DV];
            const double dR = rps * dt * msg->axes[(int)Axis::DH];

            auto& T_world_tip = robot_state->getGlobalLinkTransform(tip_link);

            auto& T_world_root = robot_state->getGlobalLinkTransform(robot_model->getRootLink());
            auto T_root_root_offset =
                    Eigen::Translation3d(dx, dy, dz);

            // world -> tip'
            // root' -> tip' = root -> tip
            // root -> world * world -> tip
            Eigen::Affine3d target_pose =
                    T_world_root *
                    T_root_root_offset *
                    T_world_root.inverse() *
                    T_world_tip *
                    Eigen::AngleAxisd(dY, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(dP, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(dR, Eigen::Vector3d::UnitX());
            if (m_model->setFromIK(jg, target_pose)) {

            }
        }
    }

    for (int i = 0; i < (int)Button::Count; ++i) {
        Button b = (Button)i;
        if (ButtonPressed(*m_prev_joy, *msg, b)) {
            for (auto& e : button_press_callbacks_) {
                e.second(b);
            }
        }
    }

    m_prev_joy = msg;
}

void TeleopCommand::cycleActiveJoint(bool forward)
{
    ROS_DEBUG_STREAM_NAMED(LOG, "Cycle to next joint variable of group " << m_active_group_name);

    auto& robot_model = m_model->getRobotModel();
    auto* jg = robot_model->getJointModelGroup(m_active_group_name);
    if (!jg) {
        return;
    }

    ROS_DEBUG_STREAM_NAMED(LOG, "  Prev variable index: " << m_remote_curr_var);

    if (forward) {
        m_remote_curr_var++;
        m_remote_curr_var %= jg->getVariableCount();
    } else {
        m_remote_curr_var--;
        if (m_remote_curr_var < 0) {
            m_remote_curr_var = jg->getVariableCount() - 1;
        }
    }
    ROS_DEBUG_STREAM_NAMED(LOG, "  Curr variable index: " << m_remote_curr_var);

    ROS_DEBUG_STREAM_NAMED(LOG, "Update active joint variable to '" <<
            jg->getVariableNames()[m_remote_curr_var] << "'");

    Q_EMIT updateActiveJointVariable(jg->getVariableIndexList()[m_remote_curr_var]);
}

void TeleopCommand::cycleActiveGroup(bool forward)
{
    ROS_DEBUG_STREAM_NAMED(LOG, "Cycle to next joint group");

    auto& robot_model = m_model->getRobotModel();
    assert(robot_model);

    auto& group_names = robot_model->getJointModelGroupNames();
    auto gidx = std::distance(
        begin(group_names),
        std::find(begin(group_names), end(group_names), m_active_group_name));

    if (forward) {
        gidx++;
        gidx %= robot_model->getJointModelGroups().size();
    } else {
        gidx--;
        if (gidx < 0) {
            gidx = robot_model->getJointModelGroups().size() - 1;
        }
    }

    auto* next_group = robot_model->getJointModelGroups()[gidx];
    auto& next_group_name = robot_model->getJointModelGroupNames()[gidx];
    if (m_active_group_name != next_group_name) {
        m_remote_curr_var = 0;
    }
    m_active_group_name = next_group_name;
    ROS_DEBUG_STREAM_NAMED(LOG, "  Next joint group: " << m_active_group_name);
    ROS_DEBUG_STREAM_NAMED(LOG, "  Active variable: " << next_group->getVariableNames()[0]);

    Q_EMIT updateActiveJointGroup(m_active_group_name);
}

} // namespace sbpl_interface
