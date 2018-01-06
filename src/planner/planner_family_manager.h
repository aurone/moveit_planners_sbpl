#ifndef MOVEIT_PLANNERS_SBPL_PLANNER_FAMILY_MANAGER_H
#define MOVEIT_PLANNERS_SBPL_PLANNER_FAMILY_MANAGER_H

// standard includes
#include <map>

// system includes
#include <boost/scoped_ptr.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>

namespace sbpl_interface {

class PlannerFamilyManager : public planning_interface::PlannerManager
{
public:

    typedef planning_interface::PlannerManager Base;

    PlannerFamilyManager();
    virtual ~PlannerFamilyManager();

    /// \name Reimplemented from planning_interface::PlannerManager
    ///@{

    virtual bool initialize(
        const robot_model::RobotModelConstPtr& model,
        const std::string& ns) override;

    virtual std::string getDescription() const override;

    virtual void getPlanningAlgorithms(
        std::vector<std::string>& algs) const override;

    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const override;

    virtual bool canServiceRequest(
        const planning_interface::MotionPlanRequest& req) const override;

    virtual void setPlannerConfigurations(
        const planning_interface::PlannerConfigurationMap& pcs) override;

    ///@}

private:

    typedef pluginlib::ClassLoader<planning_interface::PlannerManager>
            PlannerManagerLoader;
    typedef boost::scoped_ptr<PlannerManagerLoader> PlannerPluginLoaderPtr;
    PlannerPluginLoaderPtr m_planner_plugin_loader;

    std::map<std::string, planning_interface::PlannerManagerPtr> m_planner_plugins;

    bool parsePlannerId(
        const std::string& planner_id,
        std::string& plugin_name,
        std::string& alg_name) const;
};

MOVEIT_CLASS_FORWARD(PlannerFamilyManager);

} // namespace sbpl_interface

#endif
