#include "sbpl_planning_context.h"

namespace sbpl_interface {

SBPLPlanningContext::SBPLPlanningContext(
    const std::string& name,
    const std::string& group)
:
    Base(name, group)
{
}

SBPLPlanningContext::~SBPLPlanningContext()
{
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
    return false;
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
    return false;
}

bool SBPLPlanningContext::terminate()
{
    return true;
}

void SBPLPlanningContext::clear()
{
}


} // namespace sbpl_interface
