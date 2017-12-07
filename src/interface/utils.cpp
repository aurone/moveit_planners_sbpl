#include "utils.h"

#include <algorithm>
#include <stack>
#include <utility>

#include <leatherman/print.h>

namespace sbpl_interface {

bool ComputeAxisAlignedBoundingBox(
    const moveit::core::LinkModel& link,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size)
{
    if (link.getShapes().size() != link.getCollisionOriginTransforms().size()) {
        return false;
    }

    pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d bb_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bb_max = Eigen::Vector3d::Zero();

    for (size_t sidx = 0; sidx < link.getShapes().size(); ++sidx) {
        const auto& shape = link.getShapes()[sidx];
        const auto& shape_transform = link.getCollisionOriginTransforms()[sidx];
        Eigen::Vector3d shape_bb_pos;
        Eigen::Vector3d shape_bb_size;
        if (!ComputeAxisAlignedBoundingBox(*shape, shape_bb_pos, shape_bb_size)) {
            return false;
        }

        // create the vertices of the axis-aligned bounding box of the shape
        Eigen::Vector3d corners[8] = {
            Eigen::Vector3d(
                    shape_bb_pos.x() - 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() - 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() - 0.5 * shape_bb_size.z()),
            Eigen::Vector3d(
                    shape_bb_pos.x() - 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() - 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() + 0.5 * shape_bb_size.z()),
            Eigen::Vector3d(
                    shape_bb_pos.x() - 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() + 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() - 0.5 * shape_bb_size.z()),
            Eigen::Vector3d(
                    shape_bb_pos.x() - 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() + 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() + 0.5 * shape_bb_size.z()),
            Eigen::Vector3d(
                    shape_bb_pos.x() + 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() - 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() - 0.5 * shape_bb_size.z()),
            Eigen::Vector3d(
                    shape_bb_pos.x() + 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() - 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() + 0.5 * shape_bb_size.z()),
            Eigen::Vector3d(
                    shape_bb_pos.x() + 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() + 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() - 0.5 * shape_bb_size.z()),
            Eigen::Vector3d(
                    shape_bb_pos.x() + 0.5 * shape_bb_size.x(),
                    shape_bb_pos.y() + 0.5 * shape_bb_size.y(),
                    shape_bb_pos.z() + 0.5 * shape_bb_size.z())
        };

        // transform into the link frame
        for (int i = 0; i < 8; ++i) {
            corners[i] = shape_transform * corners[i];
        }

        // update the overall bounding box
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3d& corner = corners[i];
            if (sidx == 0 && i == 0) {
                bb_min = corner;
                bb_max = corner;
            } else {
                bb_min.x() = std::min(bb_min.x(), corner.x());
                bb_min.y() = std::min(bb_min.y(), corner.y());
                bb_min.z() = std::min(bb_min.z(), corner.z());
                bb_max.x() = std::max(bb_max.x(), corner.x());
                bb_max.y() = std::max(bb_max.y(), corner.y());
                bb_max.z() = std::max(bb_max.z(), corner.z());
            }
        }
    }

//    ROS_INFO("%s: min: (%0.3f, %0.3f, %0.3f), max: (%0.3f, %0.3f, %0.3f)",
//            link.getName().c_str(),
//            bb_min.x(), bb_min.y(), bb_min.z(),
//            bb_max.x(), bb_max.y(), bb_max.z());
    pos = 0.5 * (bb_min + bb_max);
    size = bb_max - bb_min;
    return true;
}

bool ComputeAxisAlignedBoundingBox(
    const shapes::Shape& shape,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size)
{
    switch (shape.type) {
    case shapes::ShapeType::BOX:
    {
        const shapes::Box& box = dynamic_cast<const shapes::Box&>(shape);
        return ComputeAxisAlignedBoundingBox(box, pos, size);
    }   break;
    case shapes::ShapeType::CYLINDER:
    {
        const shapes::Cylinder& cylinder = dynamic_cast<const shapes::Cylinder&>(shape);
        return ComputeAxisAlignedBoundingBox(cylinder, pos, size);
    }   break;
    case shapes::ShapeType::SPHERE:
    {
        const shapes::Sphere& sphere = dynamic_cast<const shapes::Sphere&>(shape);
        return ComputeAxisAlignedBoundingBox(sphere, pos, size);
    }   break;
    case shapes::ShapeType::MESH:
    {
        const shapes::Mesh& mesh = dynamic_cast<const shapes::Mesh&>(shape);
        return ComputeAxisAlignedBoundingBox(mesh, pos, size);
    }   break;
    default:
        return false;
    }
}

bool ComputeAxisAlignedBoundingBox(
    const shapes::Box& box,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size)
{
    pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    size = Eigen::Vector3d(box.size[0], box.size[1], box.size[2]);
    return true;
}

bool ComputeAxisAlignedBoundingBox(
    const shapes::Cylinder& cylinder,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size)
{
    pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    size = Eigen::Vector3d(cylinder.radius, cylinder.radius, cylinder.length);
    return true;
}

bool ComputeAxisAlignedBoundingBox(
    const shapes::Sphere& sphere,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size)
{
    pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    size = Eigen::Vector3d(sphere.radius, sphere.radius, sphere.radius);
    return true;
}

bool ComputeAxisAlignedBoundingBox(
    const shapes::Mesh& mesh,
    Eigen::Vector3d& pos,
    Eigen::Vector3d& size)
{
    Eigen::Vector3d bb_min;
    Eigen::Vector3d bb_max;
    for (size_t tidx = 0; tidx < mesh.triangle_count; ++tidx) {
        const unsigned int vidx1 = mesh.triangles[3 * tidx + 0];
        const unsigned int vidx2 = mesh.triangles[3 * tidx + 1];
        const unsigned int vidx3 = mesh.triangles[3 * tidx + 2];
        const double vx1 = mesh.vertices[3 * vidx1 + 0];
        const double vy1 = mesh.vertices[3 * vidx1 + 1];
        const double vz1 = mesh.vertices[3 * vidx1 + 2];

        const double vx2 = mesh.vertices[3 * vidx2 + 0];
        const double vy2 = mesh.vertices[3 * vidx2 + 1];
        const double vz2 = mesh.vertices[3 * vidx2 + 2];

        const double vx3 = mesh.vertices[3 * vidx3 + 0];
        const double vy3 = mesh.vertices[3 * vidx3 + 1];
        const double vz3 = mesh.vertices[3 * vidx3 + 2];

        if (tidx == 0) {
            bb_min.x() = bb_max.x() = vx1;
            bb_min.y() = bb_max.y() = vy1;
            bb_min.z() = bb_max.z() = vz1;
        } else {
            bb_min.x() = std::min(bb_min.x(), vx1);
            bb_min.y() = std::min(bb_min.y(), vy1);
            bb_min.z() = std::min(bb_min.z(), vz1);

            bb_max.x() = std::max(bb_max.x(), vx1);
            bb_max.y() = std::max(bb_max.y(), vy1);
            bb_max.z() = std::max(bb_max.z(), vz1);
        }

        bb_min.x() = std::min(bb_min.x(), vx2);
        bb_min.y() = std::min(bb_min.y(), vy2);
        bb_min.z() = std::min(bb_min.z(), vz2);

        bb_min.x() = std::min(bb_min.x(), vx3);
        bb_min.y() = std::min(bb_min.y(), vy3);
        bb_min.z() = std::min(bb_min.z(), vz3);

        bb_max.x() = std::max(bb_max.x(), vx2);
        bb_max.y() = std::max(bb_max.y(), vy2);
        bb_max.z() = std::max(bb_max.z(), vz2);

        bb_max.x() = std::max(bb_max.x(), vx3);
        bb_max.y() = std::max(bb_max.y(), vy3);
        bb_max.z() = std::max(bb_max.z(), vz3);
    }

    pos = 0.5 * (bb_min + bb_max);
    size = bb_max - bb_min;
    return true;
}

bool ComputeInscribedRadius(
    const moveit::core::LinkModel& link,
    double& radius)
{
    Eigen::Vector3d pos, size;
    if (!ComputeAxisAlignedBoundingBox(link, pos, size)) {
        return false;
    }

    radius = size.x();
    radius = std::min(radius, size.y());
    radius = std::min(radius, size.z());
    radius -= pos.norm();
    return true;
}

std::vector<std::string>
GetTipLinkNames(const moveit::core::JointModelGroup& jmg)
{
    std::vector<std::string> tips;
    std::vector<const moveit::core::LinkModel*> link_tips = GetTipLinks(jmg);
    for (const moveit::core::LinkModel* link : link_tips) {
        tips.push_back(link->getName());
    }
    return tips;
}

std::vector<const moveit::core::LinkModel*>
GetTipLinks(const moveit::core::JointModelGroup& jmg)
{
    std::vector<const moveit::core::LinkModel*> tips;
    for (const moveit::core::JointModel* jm : jmg.getJointRoots()) {
        const moveit::core::LinkModel* clm = jm->getChildLinkModel();
        GetTipLinks(jmg, *clm, tips);
    }
    return tips;
}

void GetTipLinks(
    const moveit::core::JointModelGroup& jmg,
    const moveit::core::LinkModel& link,
    std::vector<const moveit::core::LinkModel*>& tips)
{
    // get child links that are part of this group
    std::vector<const moveit::core::LinkModel*> child_links;
    for (const moveit::core::JointModel* cjm : link.getChildJointModels()) {
        if (jmg.hasJointModel(cjm->getName())) {
            child_links.push_back(cjm->getChildLinkModel());
        }
    }

    if (child_links.empty() && jmg.canSetStateFromIK(link.getName())) {
        tips.push_back(&link);
    }

    for (const moveit::core::LinkModel* clm : child_links) {
        GetTipLinks(jmg, *clm, tips);
    }
}

std::ostream& operator<<(std::ostream& o, const RobotModelInfo& info)
{
    if (!info.robot) {
        o << "null";
        return o;
    }

    const auto &rm = *info.robot;

    o << "Robot Model Name: " << rm.getName() << '\n';
    o << "Robot Model Frame: " <<  rm.getModelFrame() << '\n';
    o << "Root Link Name: " << rm.getRootLinkName() << '\n';
    o << "Root Joint Name: " << rm.getRootJointName() << '\n';

    o << "--- Robot Links ---\n";
    std::stack<std::pair<int, const moveit::core::LinkModel*>> links;
    links.push(std::make_pair(0, rm.getRootLink()));
    while (!links.empty()) {
        int depth;
        const moveit::core::LinkModel* lm;
        std::tie(depth, lm) = links.top();
        links.pop();

        std::string pad(depth, ' ');
        Eigen::Vector3d bb_pos, bb_extents;
        double inscribed_radius;
        ComputeAxisAlignedBoundingBox(*lm, bb_pos, bb_extents);
        ComputeInscribedRadius(*lm, inscribed_radius);
        o << pad << lm->getName() <<
                ": Bounding Box: " <<
                "{ pos: " << "(" << bb_pos.x() << ", " << bb_pos.y() << ", " << bb_pos.z() << ")" <<
                ", size: " << "(" << bb_extents.x() << ", " << bb_extents.y() << ", " << bb_extents.z() << ")" <<
                ", radius: " << inscribed_radius << " }\n";

        for (const moveit::core::JointModel* jm : lm->getChildJointModels()) {
            links.push(std::make_pair(depth+1, jm->getChildLinkModel()));
        }
    }

    o << "--- Robot Joints ---\n";
    std::stack<std::pair<int, const moveit::core::JointModel*>> joints;
    joints.push(std::make_pair(0, rm.getRootJoint()));
    while (!joints.empty()) {
        int depth;
        const moveit::core::JointModel* jm;
        std::tie(depth, jm) = joints.top();
        joints.pop();

        std::string pad(depth, ' ');
        o << pad << jm->getName() << " (" << jm->getTypeName() << ")\n";

        const moveit::core::LinkModel* lm = jm->getChildLinkModel();
        for (const moveit::core::JointModel* j : lm->getChildJointModels()) {
            joints.push(std::make_pair(depth + 1, j));
        }
    }

    o << "--- Virtual Joints ---\n";
    auto srdf = rm.getSRDF();
    for (const auto& vj : srdf->getVirtualJoints()) {
        o << "  name: " << vj.name_ <<
                ", parent_frame: " << vj.parent_frame_ <<
                ", child_link: " << vj.child_link_ <<
                ", type: " << vj.type_ << '\n';
    }

    o << "--- Robot Joint Groups ---\n";

    const std::vector<const moveit::core::JointModelGroup*>& jmgs =
            rm.getJointModelGroups();
    for (const moveit::core::JointModelGroup* jmg : jmgs) {
        o << "Name: " << jmg->getName();
        o << "  Chain: " << (jmg->isChain() ? "true" : "false") << '\n';
        o << "  Only Single-DoF Joints: " << (jmg->isSingleDOFJoints() ? "true" : "false") << '\n';
        o << "  End Effector: " << (jmg->isEndEffector() ? "true" : "false") << '\n';
        if (jmg->isEndEffector()) {
            o << "    End Effector Name: " << jmg->getEndEffectorName() << '\n';
        }
        o << "  Maximum Extent: " << jmg->getMaximumExtent() << '\n';
        o << "  Active Joints:\n";
        for (const moveit::core::JointModel* jm : jmg->getActiveJointModels()) {
            o << "    " << jm->getName() << '\n';
        }
        o << "  Non-Active Joints:\n";
        for (const moveit::core::JointModel* jm : jmg->getJointModels()) {
            if (std::find(
                    jmg->getActiveJointModels().begin(),
                    jmg->getActiveJointModels().end(),
                    jm) ==
                jmg->getActiveJointModels().end())
            {
                o << "    " << jm->getName() << '\n';
            }
        }
        o << "  Attached End Effectors:\n";
        for (const std::string& name : jmg->getAttachedEndEffectorNames()) {
            o << "    " << name << '\n';
        }
        o << "  Common Root: " << (jmg->getCommonRoot() ? jmg->getCommonRoot()->getName().c_str() : "null") << '\n';
        o << "  Links for Setting IK:\n";
        for (const moveit::core::LinkModel* lm : jmg->getLinkModels()) {
            if (jmg->canSetStateFromIK(lm->getName())) {
                o << "    " << lm->getName() << '\n';
            }
        }
        o << "  End Effector Tips:\n";
        std::vector<const moveit::core::LinkModel*> ee_tips;
        if (jmg->getEndEffectorTips(ee_tips)) {
            for (const moveit::core::LinkModel* ee_tip : ee_tips) {
                o << "    " << ee_tip->getName() << '\n';
            }
        }
        o << "  Tip Links:\n";
        for (const std::string& tip : GetTipLinkNames(*jmg)) {
            o << "    " << tip << '\n';
        }

        auto solver = jmg->getSolverInstance();
        if (solver) {
            o << "  Kinematics Solver:\n";
            o << "    Base Frame: " << solver->getBaseFrame() << '\n';
            o << "    Default Timeout: " << solver->getDefaultTimeout() << '\n';
            o << "    Group Name: " << solver->getGroupName() << '\n';
            o << "    Joint Names: " << to_string(solver->getJointNames()) << '\n';
            o << "    getLinkNames: " << to_string(solver->getLinkNames()) << '\n';
            std::vector<unsigned int> redundant_jinds;
            solver->getRedundantJoints(redundant_jinds);
            o << "    Redundant Joint Indices: " << to_string(redundant_jinds) << '\n';
            o << "    Search Discretization: " << solver->getSearchDiscretization() << '\n';
            o << "    Tip Frames: " << to_string(solver->getTipFrames()) << '\n';
        }
    }

    o << "--- Joint Variables ---\n";

    for (size_t vind = 0; vind < rm.getVariableCount(); ++vind) {
        const std::string& var_name = rm.getVariableNames()[vind];
        const auto& var_bounds = rm.getVariableBounds(var_name);
        o << var_name << ": { bounded: " <<
                (var_bounds.position_bounded_ ? "true" : "false") <<
                ", min: " << var_bounds.min_position_ <<
                ", max: " << var_bounds.max_position_ <<
                ", vel: " << var_bounds.max_velocity_ <<
                ", acc: " << var_bounds.max_acceleration_ << " }\n";
    }

    return o;
}

std::string to_string(moveit_msgs::MoveItErrorCodes code)
{
    switch (code.val) {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::FAILURE:
        return "FAILURE";

    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
        return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
        return "PREEMPTED";

    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_PATH_CONSTRAINTS";

    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";

    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
        return "INVALID_LINK_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
        return "INVALID_OBJECT_NAME";

    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
        return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
        return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
        return "ROBOT_STATE_STALE";
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
        return "SENSOR_INFO_STALE";

    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";

    default:
        return "UNRECOGNIZED";
    }
}

} // namespace sbpl_interface
