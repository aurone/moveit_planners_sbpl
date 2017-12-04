#ifndef collision_detector_CollisionPluginSBPL_h
#define collision_detector_CollisionPluginSBPL_h

#include <moveit/collision_detection/collision_plugin.h>

namespace collision_detection {

class CollisionPluginSBPL : public CollisionPlugin
{
public:

    virtual bool initialize(
        const planning_scene::PlanningScenePtr& scene,
        bool exclusive) const;
};

} // namespace collision_detection

#endif
