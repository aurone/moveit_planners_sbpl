#include "moveit_collision_checker.h"

namespace sbpl_interface {

MoveItCollisionChecker::MoveItCollisionChecker() :
    Base()
{
}

MoveItCollisionChecker::~MoveItCollisionChecker()
{
}

bool MoveItCollisionChecker::isStateValid(
    const std::vector<double>& angles,
    bool verbose,
    bool visualize,
    double& dist)
{
    return false;
}

bool MoveItCollisionChecker::isStateToStateValid(
    const std::vector<double>& angles0,
    const std::vector<double>& angles1,
    int& path_length,
    int& num_checks,
    double& dist)
{
    return false;
}

bool MoveItCollisionChecker::interpolatePath(
    const std::vector<double>& start,
    const std::vector<double>& end,
    const std::vector<double>& inc,
    std::vector<std::vector<double>>& path)
{
    return false;
}

visualization_msgs::MarkerArray 
MoveItCollisionChecker::getCollisionModelVisualization(
    const std::vector<double>& angles)
{
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray 
MoveItCollisionChecker::getVisualization(std::string type)
{
    return visualization_msgs::MarkerArray();
}

} // namespace sbpl_interface
