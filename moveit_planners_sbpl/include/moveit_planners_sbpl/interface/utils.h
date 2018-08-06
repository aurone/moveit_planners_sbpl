#ifndef MOVEIT_PLANNERS_SBPL_UTILS_H
#define MOVEIT_PLANNERS_SBPL_UTILS_H

#include <ostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometric_shapes/shapes.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/MoveItErrorCodes.h>

namespace sbpl_interface {

bool ComputeAxisAlignedBoundingBox(
    const moveit::core::LinkModel& link,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size);

bool ComputeAxisAlignedBoundingBox(
    const shapes::Shape& shape,
    Eigen::Vector3d& pose,
    Eigen::Vector3d& size);

bool ComputeAxisAlignedBoundingBox(
    const shapes::Box& box,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size);

bool ComputeAxisAlignedBoundingBox(
    const shapes::Cylinder& cylinder,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size);

bool ComputeAxisAlignedBoundingBox(
    const shapes::Sphere& sphere,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size);

bool ComputeAxisAlignedBoundingBox(
    const shapes::Mesh& mesh,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size);

bool ComputeInscribedRadius(
    const moveit::core::LinkModel& link,
    double& radius);

// get all leaf links from which we can set the group state via ik
std::vector<std::string>
GetTipLinkNames(const moveit::core::JointModelGroup& jmg);

std::vector<const moveit::core::LinkModel*>
GetTipLinks(const moveit::core::JointModelGroup& jmg);

void GetTipLinks(
    const moveit::core::JointModelGroup& jmg,
    const moveit::core::LinkModel& link,
    std::vector<const moveit::core::LinkModel*>& tips);

// Helper struct for logging robot model info to stream with a certain format
struct RobotModelInfo
{
    moveit::core::RobotModelConstPtr robot;
    RobotModelInfo(const moveit::core::RobotModelConstPtr &robot) :
        robot(robot) { }
};

std::ostream& operator<<(std::ostream& o, const RobotModelInfo& info);

const char* to_cstring(moveit_msgs::MoveItErrorCodes code);

} // namespace sbpl_interface

#endif
