#ifndef MoveItRobotModel_h
#define MoveItRobotModel_h

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sbpl_manipulation_components/robot_model.h>

namespace sbpl_interface {

class MoveItRobotModel : public sbpl_arm_planner::RobotModel
{
public:

    MoveItRobotModel();
    virtual ~MoveItRobotModel();

    virtual bool init(
        std::string robot_description,
        std::vector<std::string>& planning_joints);

    bool init(
        const moveit::core::RobotModelConstPtr& robot_model,
        const std::string& group_name);

    virtual bool checkJointLimits(const std::vector<double>& angles);

    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        KDL::Frame& f);

    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        std::vector<double>& pose);

    virtual bool computePlanningLinkFK(
        const std::vector<double>& angles,
        std::vector<double>& pose);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        int option = 0);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<std::vector<double> >& solutions,
        int option = 0);

    virtual bool computeFastIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);

    virtual void printRobotModelInformation();

private:

    moveit::core::RobotModelConstPtr m_moveit_model;
    moveit::core::RobotStatePtr m_robot_state;
};

} // namespace sbpl_interface

#endif
