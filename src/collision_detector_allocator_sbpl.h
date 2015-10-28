#ifndef collision_detection_CollisionDetectorAllocatorSBPL_h
#define collision_detection_CollisionDetectorAllocatorSBPL_h

#include <moveit/collision_detection/collision_detector_allocator.h>
#include "collision_robot_sbpl.h"
#include "collision_world_sbpl.h"

namespace collision_detection {

class CollisionDetectorAllocatorSBPL :
    public CollisionDetectorAllocatorTemplate<
            CollisionWorldSBPL,
            CollisionRobotSBPL,
            CollisionDetectorAllocatorSBPL>
{
public:

    static const std::string NAME_;
};

} // namespace collision_detection

#endif
