#include <moveit_planners_sbpl/interface/ik_command_interactive_marker.h>

#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_planners_sbpl/interface/robot_command_model.h>
#include <moveit_planners_sbpl/interface/utils.h>

namespace sbpl_interface {

static const char* LOG = "ik_command_interactive_marker";

IKCommandInteractiveMarker::IKCommandInteractiveMarker(RobotCommandModel* model)
    : m_im_server("phantom_controls")
{
    assert(model != NULL);
    m_model = model;
    connect(m_model, SIGNAL(robotLoaded()), this, SLOT(updateRobotModel()));
    connect(m_model, SIGNAL(robotStateChanged()), this, SLOT(updateRobotState()));
}

void IKCommandInteractiveMarker::setActiveJointGroup(const std::string& group_name)
{
    if (group_name != m_active_group_name) {
        m_active_group_name = group_name;
        reinitInteractiveMarkers();
        updateInteractiveMarkers();
        Q_EMIT updateActiveJointGroup(group_name);
    }
}

static std::string MarkerNameFromTipName(const std::string& tip_name)
{
    return tip_name + "_controls";
}

static std::string TipNameFromMarkerName(const std::string& marker_name)
{
    return marker_name.substr(0, marker_name.rfind("_control"));
}

void IKCommandInteractiveMarker::updateRobotModel()
{
    reinitInteractiveMarkers();
}

void IKCommandInteractiveMarker::updateRobotState()
{
    updateInteractiveMarkers();
}

// Given a state, for each revolute joint, find the equivalent 2*pi joint
// position that is nearest (numerically) to a reference state
void CorrectIKSolution(
    moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* group,
    const moveit::core::RobotState& ref_state)
{
    for (size_t gvidx = 0; gvidx < group->getVariableCount(); ++gvidx) {
        ROS_DEBUG_NAMED(LOG, "Check variable '%s' for bounded revoluteness", group->getVariableNames()[gvidx].c_str());

        int vidx = group->getVariableIndexList()[gvidx];
        auto* joint = state.getRobotModel()->getJointOfVariable(vidx);
        if (joint->getType() != moveit::core::JointModel::REVOLUTE ||
            !joint->getVariableBounds()[0].position_bounded_)
        {
            continue;
        }

        ROS_DEBUG_NAMED(LOG, "  Normalize variable '%s'", group->getVariableNames()[gvidx].c_str());

        double spos = state.getVariablePosition(vidx);
        double vdiff = ref_state.getVariablePosition(gvidx) - spos;
        int twopi_hops = (int)std::abs(vdiff / (2.0 * M_PI));

        ROS_DEBUG_NAMED(LOG, " -> seed pos: %f", ref_state.getVariablePosition(gvidx));
        ROS_DEBUG_NAMED(LOG, " ->  sol pos: %f", spos);
        ROS_DEBUG_NAMED(LOG, " ->    vdiff: %f", vdiff);
        ROS_DEBUG_NAMED(LOG, " -> num hops: %d", twopi_hops);

        const double dir = std::copysign(1.0, vdiff);
        double npos = spos + (2.0 * M_PI) * twopi_hops * dir;
        if (std::abs(npos - ref_state.getVariablePosition(gvidx)) > M_PI) {
            npos += 2.0 * M_PI * dir;
        }

        ROS_DEBUG_NAMED(LOG, " ->     npos: %f", npos);

        if (twopi_hops) {
            ROS_DEBUG_NAMED(LOG, " -> Attempt to normalize variable '%s' to %f from %f", group->getVariableNames()[gvidx].c_str(), npos, spos);
        } else {
            ROS_DEBUG_NAMED(LOG, "No hops necessary");
        }

        state.setVariablePosition(vidx, npos);
        if (!state.satisfiesBounds(joint)) {
            ROS_WARN_NAMED(LOG, "normalized value for '%s' out of bounds",  group->getVariableNames()[gvidx].c_str());
            state.setVariablePosition(vidx, spos);
        }
    }
}

void IKCommandInteractiveMarker::processInteractiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg)
{
    ROS_DEBUG_NAMED(LOG, "Interactive marker feedback");
    ROS_DEBUG_NAMED(LOG, "  Marker: %s", msg->marker_name.c_str());
    ROS_DEBUG_NAMED(LOG, "  Control: %s", msg->control_name.c_str());
    ROS_DEBUG_NAMED(LOG, "  Event Type: %u", (unsigned)msg->event_type);

    if (msg->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        auto* robot_state = m_model->getRobotState();
        auto* group = robot_state->getJointModelGroup(m_active_group_name);
        if (!group) {
            ROS_ERROR_NAMED(LOG, "Failed to retrieve joint group '%s'", m_active_group_name.c_str());
            return;
        }

        // run ik from this tip link
        Eigen::Affine3d wrist_pose;
        tf::poseMsgToEigen(msg->pose, wrist_pose);

        moveit::core::RobotState ik_state(*robot_state);
        if (ik_state.setFromIK(group, wrist_pose)) {
            // correct solution to be closer to seed state
            CorrectIKSolution(ik_state, group, *robot_state);
            m_model->setVariablePositions(ik_state.getVariablePositions());
        } else {
            // TODO: anything special here?
        }
    }
}

// This gets called whenever the robot model or active joint group changes.
void IKCommandInteractiveMarker::reinitInteractiveMarkers()
{
    auto& robot_model = m_model->getRobotModel();

    ROS_INFO_NAMED(LOG, "Setup Interactive Markers for Robot");

    ROS_INFO_NAMED(LOG, " -> Remove any existing markers");
    m_im_server.clear();
    m_int_marker_names.clear();

    bool have_robot = (bool)robot_model;
    bool have_active_group = !m_active_group_name.empty();
    if (!have_robot || !have_active_group) {
        if (!have_robot) {
            ROS_INFO_NAMED(LOG, "No robot model to initialize interactive markers from");
        }
        if (!have_active_group) {
            ROS_INFO_NAMED(LOG, "No active joint group to initialize interactive markers from");
        }
        m_im_server.applyChanges(); // TODO: defer idiom here
        return;
    }

    auto* jg = robot_model->getJointModelGroup(m_active_group_name);
    if (!jg) {
        ROS_INFO_NAMED(LOG, "Failed to retrieve joint group '%s'", m_active_group_name.c_str());
        m_im_server.applyChanges();
        return;
    }

    auto tips = GetTipLinks(*jg);

    for (auto* tip_link : tips) {
        ROS_INFO_NAMED(LOG, "Adding interactive marker for controlling pose of link %s", tip_link->getName().c_str());

        visualization_msgs::InteractiveMarker tip_marker;
        tip_marker.header.frame_id = robot_model->getModelFrame();

        tip_marker.pose.orientation.w = 1.0;
        tip_marker.pose.orientation.x = 0.0;
        tip_marker.pose.orientation.y = 0.0;
        tip_marker.pose.orientation.z = 0.0;
        tip_marker.pose.position.x = 0.0;
        tip_marker.pose.position.y = 0.0;
        tip_marker.pose.position.z = 0.0;

        tip_marker.name = MarkerNameFromTipName(tip_link->getName());
        tip_marker.description = "ik control of link " + tip_link->getName();
        tip_marker.scale = 0.20f;

        visualization_msgs::InteractiveMarkerControl dof_control;
        dof_control.orientation_mode =
                visualization_msgs::InteractiveMarkerControl::INHERIT;
        dof_control.always_visible = false;
//        dof_control.description = "pose_control";

        dof_control.orientation.w = 1.0;
        dof_control.orientation.x = 1.0;
        dof_control.orientation.y = 0.0;
        dof_control.orientation.z = 0.0;

        dof_control.name = "rotate_x";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_x";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.orientation.w = 1.0;
        dof_control.orientation.x = 0.0;
        dof_control.orientation.y = 1.0;
        dof_control.orientation.z = 0.0;

        dof_control.name = "rotate_z";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_z";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.orientation.w = 1.0;
        dof_control.orientation.x = 0.0;
        dof_control.orientation.y = 0.0;
        dof_control.orientation.z = 1.0;

        dof_control.name = "rotate_y";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_y";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        auto feedback_fn = [this](
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg)
        {
            return this->processInteractiveMarkerFeedback(msg);
        };
        m_im_server.insert(tip_marker, feedback_fn);
        m_int_marker_names.push_back(tip_marker.name);
    }

    m_im_server.applyChanges();
}

void IKCommandInteractiveMarker::updateInteractiveMarkers()
{
    auto* robot_state = m_model->getRobotState();
    assert(robot_state != NULL);
    for (auto& marker_name : m_int_marker_names) {
        // stuff the current pose
        std::string tip_link_name = TipNameFromMarkerName(marker_name);
        auto& T_model_tip = robot_state->getGlobalLinkTransform(tip_link_name);

        geometry_msgs::Pose tip_pose;
        tf::poseEigenToMsg(T_model_tip, tip_pose);

        // update the pose of the interactive marker
        std_msgs::Header header;
        header.frame_id = m_model->getRobotModel()->getModelFrame();
        header.stamp = ros::Time(0);
        if (!m_im_server.setPose(marker_name, tip_pose, header)) {
            ROS_INFO_NAMED(LOG, "Failed to set pose of interactive marker '%s'", marker_name.c_str());
        }
    }

    m_im_server.applyChanges();
}

} // namespace sbpl_interface
