#ifndef sbpl_interface_move_group_command_model_h
#define sbpl_interface_move_group_command_model_h

// standard includes
#include <map>
#include <memory>
#include <string>

// system includes
#include <QtGui>
#include <actionlib/client/simple_action_client.h>
#include <boost/logic/tribool.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <rviz/config.h>

namespace sbpl_interface {

class MoveGroupCommandModel : public QObject
{
    Q_OBJECT

public:

    static constexpr double DefaultGoalPositionTolerance_m = 0.05;
    static constexpr double DefaultGoalOrientationTolerance_deg = 10.0;
    static constexpr double DefaultGoalJointTolerance_deg = 5.0;
    static const int DefaultNumPlanningAttempts = 1;
    static constexpr double DefaultAllowedPlanningTime_s = 10.0;

    MoveGroupCommandModel(QObject* parent = 0);

    /// \brief Load a robot into the command model.
    ///
    /// If the robot model fails load from the given robot_description, the
    /// previous model remains and no robotLoaded signal is emitted.
    ///
    /// \param robot_description The name of the ROS parameter containing the
    ///     URDF and SRDF. The SRDF is derived from robot_description +
    ///     "_semantic".
    bool loadRobot(const std::string& robot_description);

    bool isRobotLoaded() const;

    /// \brief Return the model of the commanded robot.
    moveit::core::RobotModelConstPtr robotModel() const;

    /// \brief Return the state of the phantom robot used for commanding.
    moveit::core::RobotStateConstPtr robotState() const;

    boost::tribool robotStateValidity() const { return m_validity; }

    bool readyToPlan() const;

    bool planToGoalPose(const std::string& group_name);
    bool planToGoalConfiguration(const std::string& group_name);
    bool moveToGoalPose(const std::string& group_name);
    bool moveToGoalConfiguration(const std::string& group_name);

    bool copyCurrentState();

    const std::vector<moveit_msgs::PlannerInterfaceDescription>&
    plannerInterfaces() const;

    /// \name General/Robot Settings
    ///@{
    const std::string robotDescription() const;
    ///@}

    /// \name Planner Settings
    ///@{
    const std::string plannerName() const;
    const std::string plannerID() const;
    int numPlanningAttempts() const;
    double allowedPlanningTime() const;
    ///@}

    /// \name Goal Constraints Settings
    ///@{
    const std::string& planningJointGroupName() const;
    double goalJointTolerance() const;
    double goalPositionTolerance() const;
    double goalOrientationTolerance() const;
    ///@}

    void load(const rviz::Config& config);
    void save(rviz::Config config) const;

public Q_SLOTS:

    void setPlannerName(const std::string& planner_name);
    void setPlannerID(const std::string& planner_id);
    void setNumPlanningAttempts(int num_planning_attempts);
    void setAllowedPlanningTime(double allowed_planning_time_s);
    void setPlanningJointGroup(const std::string& joint_group_name);
    void setJointVariable(int jidx, double value);
    void setJointVariable(const std::string& jv_name, double value);
    void setGoalJointTolerance(double tol_deg);
    void setGoalPositionTolerance(double tol_m);
    void setGoalOrientationTolerance(double tol_deg);

Q_SIGNALS:

    void robotLoaded();
    void robotStateChanged();

    /// \brief Signal that a configuration setting has been modified
    ///
    /// The following setting changes are signalled by this signal:
    ///     * planner name
    ///     * planner id
    ///     * num planning attempts
    ///     * allowed planning time
    ///     * active planning joint group
    ///     * any goal constraint tolerance
    ///     * workspace boundaries
    void configChanged();

private:

    // assertions:
    // * robot_loaded:
    //     m_scene_monitor ^ !robotDescription().empty() ^
    //     robotModel() ^ robotState()
    // * (robot loaded and model has at least one joint group) ^ active joint group is non-empty

    ros::NodeHandle m_nh;

    planning_scene_monitor::PlanningSceneMonitorPtr m_scene_monitor;

    moveit::core::RobotStatePtr m_robot_state;

    boost::tribool m_validity;

    /// \name move_group commands
    ///@{
    std::unique_ptr<ros::ServiceClient> m_check_state_validity_client;
    std::unique_ptr<ros::ServiceClient> m_query_planner_interface_client;

    typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupActionClient;
    std::unique_ptr<MoveGroupActionClient> m_move_group_client;
    ///@}

    std::vector<moveit_msgs::PlannerInterfaceDescription> m_planner_interfaces;
    int m_curr_planner_idx;
    int m_curr_planner_id_idx;

    /// \name MotionPlanRequest settings
    ///@{
    double m_joint_tol_rad;
    double m_pos_tol_m;
    double m_rot_tol_rad;

    int m_num_planning_attempts;
    double m_allowed_planning_time_s;

    std::string m_curr_joint_group_name;
    ///@}

    interactive_markers::InteractiveMarkerServer m_im_server;
    std::vector<std::string> m_int_marker_names;

    void reinitCheckStateValidityService();
    void reinitQueryPlannerInterfaceService();

    void logRobotModelInfo(const moveit::core::RobotModel& rm) const;
    void logPlanningSceneMonitor(
        const planning_scene_monitor::PlanningSceneMonitor& monitor) const;

    void reinitInteractiveMarkers();

    // Synchronize the poses of all markers with the current robot state.
    void updateInteractiveMarkers();

    void updateRobotStateValidity();

    void clearMoveGroupRequest();

    bool fillWorkspaceParameters(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req);
    bool fillStartState(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillGoalConstraints(
        const ros::Time& now,
        const std::string& group,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillPathConstraints(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillTrajectoryConstraints(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;

    void logMotionPlanResponse(
        const moveit_msgs::MotionPlanResponse& res) const;
    void logMotionPlanResponse(
        const moveit_msgs::MoveGroupResult& res) const;

    bool sendMoveGroupPoseGoal(
        const std::string& group_name,
        const moveit_msgs::PlanningOptions& ops);

    void moveGroupResultCallback(
        const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::MoveGroupResult::ConstPtr& result);

    // Get the state of the real robot.
    //
    // Returns false if the current robot state is not available. \p robot_state
    // is not modified if the current state is not available.
    bool getActualState(moveit::core::RobotState& robot_state) const;

    std::vector<std::string>
    getTipLinks(const moveit::core::JointModelGroup& jmg) const;

    void getTipLinks(
        const moveit::core::JointModelGroup& jmg,
        const moveit::core::LinkModel& link,
        std::string& tip,
        std::vector<std::string>& tips) const;

    void processInteractiveMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);

    std::string markerNameFromTipName(const std::string& tip_name) const;
    std::string tipNameFromMarkerName(const std::string& marker_name) const;

    bool plannerIndicesValid(int planner_idx, int planner_id_idx) const;

    bool hasVariable(
        const moveit::core::RobotModel& rm,
        const std::string& jv_name) const;
};

} // namespace sbpl_interface

#endif
