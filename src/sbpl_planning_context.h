#ifndef sbpl_interface_SBPLPlanningContext_h
#define sbpl_interface_SBPLPlanningContext_h

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>

namespace sbpl_interface {

class SBPLPlanningContext : public planning_interface::PlanningContext
{
public:

    typedef planning_interface::PlanningContext Base;

    SBPLPlanningContext(const std::string& name, const std::string& group);
    virtual ~SBPLPlanningContext();

    virtual bool solve(planning_interface::MotionPlanResponse& res);
    virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);
    virtual bool terminate();
    virtual void clear();
};

MOVEIT_CLASS_FORWARD(SBPLPlanningContext);

} // namespace sbpl_interface

#endif
