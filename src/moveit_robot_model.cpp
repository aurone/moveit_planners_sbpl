#include "moveit_robot_model.h"

namespace sbpl_interface {

MoveItRobotModel::MoveItRobotModel()
{

}

MoveItRobotModel::~MoveItRobotModel()
{

}

bool MoveItRobotModel::init(
    std::string robot_description,
    std::vector<std::string>& planning_joints)
{
    return false;
}

bool MoveItRobotModel::init(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name)
{
    return false;
}

bool MoveItRobotModel::checkJointLimits(const std::vector<double>& angles)
{
    return false;
}

bool MoveItRobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    KDL::Frame& f)
{
    return false;
}

bool MoveItRobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    std::vector<double>& pose)
{
    return false;
}

bool MoveItRobotModel::computePlanningLinkFK(
    const std::vector<double>& angles,
    std::vector<double>& pose)
{
    return false;
}

bool MoveItRobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution,
    int option)
{
    return false;
}

bool MoveItRobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<std::vector<double> >& solutions,
    int option)
{
    return false;
}

bool MoveItRobotModel::computeFastIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution)
{
    return false;
}

void MoveItRobotModel::printRobotModelInformation()
{

}

} // namespace sbpl_interface
