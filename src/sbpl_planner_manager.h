#ifndef sbpl_interface_sbpl_planner_manager_h
#define sbpl_interface_sbpl_planner_manager_h

#include <XmlRpcValue.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_planners_sbpl/moveit_robot_model.h>

#include <smpl/debug/visualizer_ros.h>

namespace sbpl_interface {

class SBPLPlannerManager : public planning_interface::PlannerManager
{
public:

    static const std::string DefaultPlanningAlgorithm;

    typedef planning_interface::PlannerManager Base;

    SBPLPlannerManager();
    virtual ~SBPLPlannerManager();

    /// \name planning_interface::PlannerManager API Requirements
    ///@{

    /// \sa planning_interface::PlannerManager::initialize()
    virtual bool initialize(
        const robot_model::RobotModelConstPtr& model,
        const std::string& ns) override;

    /// \sa planning_interface::PlannerManger::getDescription()
    virtual std::string getDescription() const;

    /// \sa planning_interface::PlannerManager:::getPlanningAlgorithms()
    virtual void getPlanningAlgorithms(
        std::vector<std::string>& algs) const override;

    /// \sa planning_interface::PlannerManager::getPlanningContext()
    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const override;

    /// \sa planning_interface::PlannerManager::canServiceRequest()
    virtual bool canServiceRequest(
        const planning_interface::MotionPlanRequest& req) const override;

    /// \sa planning_interface::PlannerManager::setPlannerConfigurations()
    virtual void setPlannerConfigurations(
        const planning_interface::PlannerConfigurationMap& pcs) override;

    ///@}

private:

    moveit::core::RobotModelConstPtr m_robot_model;
    std::string m_ns;

    // per-group sbpl robot model
    std::map<std::string, std::shared_ptr<MoveItRobotModel>> m_sbpl_models;

    // per-group sbpl collision allocators
    typedef collision_detection::CollisionDetectorAllocatorPtr
    CollisionCheckerAllocatorPtr;
    std::map<std::string, CollisionCheckerAllocatorPtr> m_cc_allocators;

    sbpl::VisualizerROS m_viz;

    void logPlanningScene(const planning_scene::PlanningScene& scene) const;
    void logMotionPlanRequest(
        const planning_interface::MotionPlanRequest& req) const;

    /// \name Parameter Loading
    ///@{

    bool loadPlannerConfigurationMapping(const moveit::core::RobotModel& model);

    // map from planner configuration name to planner settings
    // (set of (name, value)) pairs
    typedef std::map<std::string, std::string> PlannerSettings;
    typedef std::map<std::string, PlannerSettings> PlannerSettingsMap;
    bool loadPlannerSettings(PlannerSettingsMap& planner_settings);

    bool xmlToString(XmlRpc::XmlRpcValue& value, std::string& str) const;

    ///@}

    // retrive an already-initialized model for a given group
    MoveItRobotModel* getModelForGroup(const std::string& group_name);

    std::string selectPlanningLink(
        const planning_interface::MotionPlanRequest& req) const;
};

MOVEIT_CLASS_FORWARD(SBPLPlannerManager);

} // namespace sbpl_interface

#endif
