#ifndef collision_detection_CollisionRobotSBPL_h
#define collision_detection_CollisionRobotSBPL_h

#include <moveit/collision_detection/collision_robot.h>

namespace collision_detection {

class CollisionRobotSBPL : public CollisionRobot
{
public:

    CollisionRobotSBPL(
        const robot_model::RobotModelConstPtr& model,
        double padding = 0.0,
        double scale = 1.0);
    CollisionRobotSBPL(const CollisionRobotSBPL& other);

    virtual ~CollisionRobotSBPL();

    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& robot_state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state) const;

    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& robot_state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state1,
        const robot_state::RobotState& other_state2) const;

    virtual void checkOtherCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state1,
        const robot_state::RobotState& other_state2,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2) const;

    virtual void checkSelfCollision(
        const CollisionRequest& req,
        CollisionResult& res,
        const robot_state::RobotState& state1,
        const robot_state::RobotState& state2,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceOther(
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state) const;

    virtual double distanceOther(
        const robot_state::RobotState& state,
        const CollisionRobot& other_robot,
        const robot_state::RobotState& other_state,
        const AllowedCollisionMatrix& acm) const;

    virtual double distanceSelf(
        const robot_state::RobotState& state) const;

    virtual double distanceSelf(
        const robot_state::RobotState& state,
        const AllowedCollisionMatrix& acm) const;

protected:

    virtual void updatedPaddingOrScaling(const std::vector<std::string>& links);

private:

    void clearAllCollisions(CollisionResult& res) const;
};

} // namespace collision_detection

#endif
