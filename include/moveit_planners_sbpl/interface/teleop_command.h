#ifndef MOVEIT_PLANNERS_SBPL_TELEOP_COMMAND_H
#define MOVEIT_PLANNERS_SBPL_TELEOP_COMMAND_H

#include <functional>
#include <map>

#include <QObject>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class QTimer;

namespace sbpl_interface {

class RobotCommandModel;

class TeleopCommand : public QObject
{
    Q_OBJECT

public:

    enum struct Button {
        A = 0,      // next joint
        B,          // next joint group
        X,          // prev joint
        Y,          // previous joint group
        LB,
        RB,
        Back,       // set active group to zero state
        Start,
        Power,
        LStick,
        RStick,
        Count,
    };

    enum struct Axis {
        LH = 0, // left horizontal     - ik yaw left/right
        LV,     // left vertical       - ik up/down
        LT,     // left trigger        - joint down
        RH,     // right horizontal    - ik left/right
        RV,     // right vertical      - ik forward/back
        RT,     // right trigger       - joint up
        DH,     // d-pad horizontal    - roll
        DV,     // d-pad vertical      - pitch
        Count,
    };

    enum struct ButtonState {
        Up = 0,
        Down = 1,
    };

    // TODO: what is most convenient here?
    // * single callback called multiple times for each button press?
    // * single callback handled entire mask of button states
    // * single callback per associated button press/release
    using ButtonPressCallback = std::function<void(Button)>;
    using ButtonReleaseCallback = std::function<void(Button)>;

    TeleopCommand(RobotCommandModel* model, const std::string& joy_topic);

    auto getActiveJointGroup() const -> const std::string& {
        return m_active_group_name;
    }

    size_t registerButtonPressCallback(const ButtonPressCallback& cb);

    void unregisterButtonPressCallback(size_t handle);

public Q_SLOTS:

    void setActiveJointGroup(const std::string& group_name);

Q_SIGNALS:

    void updateActiveJointGroup(const std::string& group_name);
    void updateActiveJointVariable(int vidx);

private:

    RobotCommandModel* m_model = nullptr;
    std::string m_active_group_name;


    ros::NodeHandle m_nh;
    ros::Subscriber m_joy_sub;

    sensor_msgs::Joy::ConstPtr m_init_joy;

    std::vector<bool> m_pressed;
    std::vector<bool> m_released;

    sensor_msgs::Joy::ConstPtr m_prev_joy;
    sensor_msgs::Joy::ConstPtr m_last_processed;
    int m_remote_curr_var = -1;

    // ordered map for handle maintenance
    std::map<size_t, ButtonPressCallback> button_press_callbacks_;

    QTimer* m_timer;

private Q_SLOTS:

    bool nearInitAxis(
        const sensor_msgs::Joy::ConstPtr& joy,
        TeleopCommand::Axis axis) const;

    void updateRobotModel();
    void updateRobotState();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void update();

    void cycleActiveJoint(bool forward);
    void cycleActiveGroup(bool forward);
    void moveActiveJoint(double dp);
};

} // namespace sbpl_interface

#endif
