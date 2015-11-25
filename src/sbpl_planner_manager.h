#ifndef sbpl_interface_SBPLPlannerManager_h
#define sbpl_interface_SBPLPlannerManager_h

#include <XmlRpcValue.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_planners_sbpl/moveit_robot_model.h>

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
        const std::string& ns);

    /// \sa planning_interface::PlannerManger::getDescription()
    virtual std::string getDescription() const;

    /// \sa planning_interface::PlannerManager:::getPlanningAlgorithms()
    void getPlanningAlgorithms(std::vector<std::string>& algs) const;

    /// \sa planning_interface::PlannerManager::getPlanningContext()
    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const;

    /// \sa planning_interface::PlannerManager::canServiceRequest()
    virtual bool canServiceRequest(
        const planning_interface::MotionPlanRequest& req) const;

    /// \sa planning_interface::PlannerManager::setPlannerConfigurations()
    virtual void setPlannerConfigurations(
        const planning_interface::PlannerConfigurationMap& pcs);

    /// \sa planning_interface::PlannerManager::terminate()
    void terminate() const;

    ///@}

private:

    moveit::core::RobotModelConstPtr m_robot_model;
    std::string m_ns;

    bool m_use_sbpl_cc;

    // per-group sbpl robot model
    std::map<std::string, MoveItRobotModel> m_sbpl_models;

    // per-group sbpl collision allocators
    typedef collision_detection::CollisionDetectorAllocatorPtr
    CollisionCheckerAllocatorPtr;
    std::map<std::string, CollisionCheckerAllocatorPtr> m_cc_allocators;

    void logPlanningScene(const planning_scene::PlanningScene& scene) const;
    void logMotionRequest(
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

    bool selectCollisionCheckerSBPL(
        planning_scene::PlanningScene& scene,
        const MoveItRobotModel* sbpl_robot_model,
        const std::string& group_name);

    bool initializeCollisionWorld(
        planning_scene::PlanningScene& scene,
        const MoveItRobotModel* sbpl_robot_model,
        const std::string& group_name);
};

MOVEIT_CLASS_FORWARD(SBPLPlannerManager);

} // namespace sbpl_interface

#endif
