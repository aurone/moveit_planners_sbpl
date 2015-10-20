#ifndef sbpl_interface_MoveItCollisionChecker_h
#define sbpl_interface_MoveItCollisionChecker_h

#include <sbpl_manipulation_components/collision_checker.h>

namespace sbpl_interface {

class MoveItCollisionChecker : public sbpl_arm_planner::CollisionChecker
{
public:

    typedef sbpl_arm_planner::CollisionChecker Base;

    MoveItCollisionChecker();
    ~MoveItCollisionChecker();

    bool isStateValid(
        const std::vector<double>& angles,
        bool verbose,
        bool visualize,
        double& dist);

    bool isStateToStateValid(
        const std::vector<double>& angles0,
        const std::vector<double>& angles1,
        int& path_length,
        int& num_checks,
        double& dist);

    bool interpolatePath(
        const std::vector<double>& start,
        const std::vector<double>& end,
        const std::vector<double>& inc,
        std::vector<std::vector<double>>& path);

    visualization_msgs::MarkerArray getCollisionModelVisualization(
        const std::vector<double>& angles);

    visualization_msgs::MarkerArray getVisualization(std::string type);

private:
};

} // namespace sbpl_interface

#endif
