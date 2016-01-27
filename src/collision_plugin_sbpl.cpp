#include "collision_plugin_sbpl.h"

#include <moveit_planners_sbpl/collision_detector_allocator_sbpl.h>

namespace collision_detection {

bool CollisionPluginSBPL::initialize(
    const planning_scene::PlanningScenePtr& scene,
    bool exclusive) const
{
    scene->setActiveCollisionDetector(
            collision_detection::CollisionDetectorAllocatorSBPL::create(),
            exclusive);
    return true;
}

} // namespace collision_detection

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionPluginSBPL, collision_detection::CollisionPlugin);
