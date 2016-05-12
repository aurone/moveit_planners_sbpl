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

    const std::string& robotDescription() const;
    moveit::core::RobotModelConstPtr robotModel() const;
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

    const std::string plannerName() const;
    const std::string plannerID() const;

    double goalJointTolerance() const;
    double goalPositionTolerance() const;
    double goalOrientationTolerance() const;

    int numPlanningAttempts() const;
    double allowedPlanningTime() const;

public Q_SLOTS:

    void setJointVariable(int jidx, double value);
    void setGoalJointTolerance(double tol_deg);
    void setGoalPositionTolerance(double tol_m);
    void setGoalOrientationTolerance(double tol_deg);
    void setPlannerName(const std::string& planner_name);
    void setPlannerID(const std::string& planner_id);
    void setNumPlanningAttempts(int num_planning_attempts);
    void setAllowedPlanningTime(double allowed_planning_time_s);

Q_SIGNALS:

    void robotLoaded();
    void robotStateChanged();
    void readyStatusChanged();

private:

    ros::NodeHandle m_nh;

    // robot model
    std::string m_robot_description;
    robot_model_loader::RobotModelLoaderPtr m_rm_loader;
    moveit::core::RobotModelPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    boost::tribool m_validity;

    planning_scene_monitor::PlanningSceneMonitorPtr m_scene_monitor;

    // move_group API
    std::unique_ptr<ros::ServiceClient> m_check_state_validity_client;
    std::unique_ptr<ros::ServiceClient> m_query_planner_interface_client;

    typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupActionClient;
    std::unique_ptr<MoveGroupActionClient> m_move_group_client;

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
    ///@}

    interactive_markers::InteractiveMarkerServer m_im_server;

    void reinitCheckStateValidityService();
    void reinitQueryPlannerInterfaceService();

    void logRobotModelInfo(const moveit::core::RobotModel& rm) const;
    void logPlanningSceneMonitor(
        const planning_scene_monitor::PlanningSceneMonitor& monitor) const;

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

    // Get the state of the real robot, if it's available
    bool getActualState(moveit::core::RobotState& robot_state) const;

    std::vector<std::string>
    getTipLinks(const moveit::core::JointModelGroup& jmg) const;

    void getTipLinks(
        const moveit::core::JointModelGroup& jmg,
        const moveit::core::LinkModel& link,
        std::string& tip,
        std::vector<std::string>& tips) const;
};

} // namespace sbpl_interface

#endif
