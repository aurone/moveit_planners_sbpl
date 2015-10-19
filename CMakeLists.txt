cmake_minimum_required(VERSION 2.8.3)
project(moveit_planners_sbpl)

find_package(sbpl REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS moveit_core moveit_ros_planning roscpp pluginlib)

catkin_package()

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    moveit_planners_sbpl 
    src/sbpl_planner_manager.cpp
    src/sbpl_planning_context.cpp)
target_link_libraries(
    moveit_planners_sbpl 
    ${catkin_LIBRARIES} 
    ${SBPL_LIBRARIES})

install(
    FILES sbpl_interface_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    TARGETS moveit_planners_sbpl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
