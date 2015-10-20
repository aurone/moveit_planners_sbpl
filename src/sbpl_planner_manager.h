#ifndef sbpl_interface_SBPLPlannerManager_h
#define sbpl_interface_SBPLPlannerManager_h

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>

namespace sbpl_interface {

class SBPLPlannerManager : public planning_interface::PlannerManager
{
public:

    static const std::string DefaultPlanningAlgorithm;

    typedef planning_interface::PlannerManager Base;

    SBPLPlannerManager();
    virtual ~SBPLPlannerManager();

    virtual bool initialize(
        const robot_model::RobotModelConstPtr& model,
        const std::string& ns);

    virtual std::string getDescription() const;

    void getPlanningAlgorithms(std::vector<std::string>& algs) const;

    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const;

    virtual bool canServiceRequest(
        const planning_interface::MotionPlanRequest& req) const;

    virtual void setPlannerConfigurations(
        const planning_interface::PlannerConfigurationMap& pcs);

    void terminate() const;

private:

    moveit::core::RobotModelConstPtr m_robot_model;
    std::string m_ns;

    void logPlanningScene(const planning_scene::PlanningScene& scene) const;
};

MOVEIT_CLASS_FORWARD(SBPLPlannerManager);

} // namespace sbpl_interface

#endif
