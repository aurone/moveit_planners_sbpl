#ifndef collision_detection_CollisionWorldSBPL_h
#define collision_detection_CollisionWorldSBPL_h

#include <memory>

#include <moveit/collision_detection/collision_world.h>

#include <sbpl_collision_checking/sbpl_collision_space.h>

namespace collision_detection {

class CollisionWorldSBPL : public CollisionWorld
{
public:

    CollisionWorldSBPL();
    CollisionWorldSBPL(const WorldPtr& world);
    CollisionWorldSBPL(const CollisionWorldSBPL& other, const WorldPtr& world);

    virtual ~CollisionWorldSBPL();

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state) const;

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2) const;

    virtual void checkRobotCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionRobot& robot,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkWorldCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionWorld& other_world) const;

    virtual void checkWorldCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const CollisionWorld& other_world,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state) const;

    virtual double distanceRobot(
        const CollisionRobot& robot,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceWorld(const CollisionWorld& world) const;
    
    virtual double distanceWorld(
        const CollisionWorld& world,
        const AllowedCollisionMatrix& acm) const;

    virtual void setWorld(const WorldPtr& world);

    bool init(
        const std::string& urdf_string,
        const std::string& group_name,
        const sbpl::collision::CollisionModelConfig& config);

private:

    std::unique_ptr<distance_field::PropagationDistanceField> m_dfield;
    std::unique_ptr<sbpl_arm_planner::OccupancyGrid> m_grid;
    std::unique_ptr<sbpl::collision::SBPLCollisionSpace> m_cspace;

    World::ObserverHandle m_observer_handle;

    bool initialized() const;
    void registerWorldCallback();
    void worldUpdate(const World::ObjectConstPtr& object, World::Action action);

    bool checkDegenerateCollision(CollisionResult& res) const;
    void setVacuousCollision(CollisionResult& res) const;
    void clearAllCollisions(CollisionResult& res) const;
};

} // namespace collision_detection

#endif
