#include "planner_family_manager.h"

#include <ros/ros.h>

namespace sbpl_interface {

PlannerFamilyManager::PlannerFamilyManager() :
    Base()
{
}

PlannerFamilyManager::~PlannerFamilyManager()
{
}

bool PlannerFamilyManager::initialize(
    const robot_model::RobotModelConstPtr& model,
    const std::string& ns)
{
    ros::NodeHandle nh(ns);

    XmlRpc::XmlRpcValue planner_plugins_cfg;
    if (!nh.getParam("planner_plugins", planner_plugins_cfg)) {
        ROS_ERROR("Failed to retrieve 'planner_plugins' from the param server");
        return false;
    }
    if (planner_plugins_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("'planner_plugins' param must be a struct");
        return false;
    }

    std::map<std::string, std::string> planner_plugins;
    for (auto it = planner_plugins_cfg.begin(); it != planner_plugins_cfg.end(); ++it) {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("'planner_plugins' element value must be a string naming a planner plugin");
            return false;
        }

        planner_plugins.insert(std::make_pair(it->first, std::string(it->second)));
    }

    ROS_INFO("Planner Plugins:");
    for (const auto& ent : planner_plugins) {
        ROS_INFO("  %s: %s", ent.first.c_str(), ent.second.c_str());
    }

    try {
        m_planner_plugin_loader.reset(new PlannerManagerLoader(
                "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    std::vector<std::string> classes = m_planner_plugin_loader->getDeclaredClasses();
    for (const auto& ent : planner_plugins) {
        if (std::find(classes.begin(), classes.end(), ent.second) != classes.end()) {
            planning_interface::PlannerManagerPtr planner_plugin;
            planner_plugin.reset(m_planner_plugin_loader->createUnmanagedInstance(ent.second));

            if (!planner_plugin->initialize(model, ns)) {
                ROS_WARN("Failed to initialize planner plugin '%s'", ent.second.c_str());
            }
            else {
                m_planner_plugins.insert(std::make_pair(ent.first, std::move(planner_plugin)));
            }
        }
        else {
            ROS_WARN("Did not find planner plugin '%s' in the list of available plugins", ent.second.c_str());
            continue;
        }
    }
    return true;
}

std::string PlannerFamilyManager::getDescription() const
{
    return "A family of algorithms for planning";
}

void PlannerFamilyManager::getPlanningAlgorithms(
    std::vector<std::string>& algs) const
{
    algs.clear();
    for (const auto& ent : m_planner_plugins) {
        std::vector<std::string> plugin_algs;
        ent.second->getPlanningAlgorithms(plugin_algs);
        for (const std::string& alg : plugin_algs) {
            algs.push_back(ent.first + "." + alg);
        }
    }
}

planning_interface::PlanningContextPtr PlannerFamilyManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    std::string plugin_name, alg_name;
    if (!parsePlannerId(req.planner_id, plugin_name, alg_name)) {
        ROS_ERROR("Failed to parse planner id for plugin name");
        return planning_interface::PlanningContextPtr();
    }

    planning_interface::MotionPlanRequest mreq = req;
    mreq.planner_id = alg_name;
    return m_planner_plugins.at(plugin_name)->getPlanningContext(
            planning_scene, mreq, error_code);
}

bool PlannerFamilyManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    std::string plugin_name, alg_name;
    if (!parsePlannerId(req.planner_id, plugin_name, alg_name)) {
        return false;
    }

    planning_interface::MotionPlanRequest mreq = req;
    mreq.planner_id = alg_name;
    return m_planner_plugins.at(plugin_name)->canServiceRequest(req);
}

void PlannerFamilyManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pcs)
{
}

bool PlannerFamilyManager::parsePlannerId(
    const std::string& planner_id,
    std::string& plugin_name,
    std::string& alg_name) const
{
    const size_t delim_pos = planner_id.find_first_of('.');
    if (delim_pos == std::string::npos) {
        return false;
    }
    std::string plug_name = planner_id.substr(0, delim_pos);
    if (m_planner_plugins.find(plug_name) != m_planner_plugins.end()) {
        plugin_name = plug_name;
        alg_name = planner_id.substr(delim_pos + 1);
        return true;
    }
    else {
        return false;
    }
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
        sbpl_interface::PlannerFamilyManager,
        planning_interface::PlannerManager);
