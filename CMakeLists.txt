cmake_minimum_required(VERSION 2.8.3)
project(moveit_planners_sbpl)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(sbpl REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS 
    moveit_core 
    moveit_ros_planning 
    roscpp 
    sbpl_manipulation 
    pluginlib)

catkin_package()

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    moveit_sbpl_planner_plugin 
    src/sbpl_planner_manager.cpp
    src/sbpl_planning_context.cpp
    src/moveit_robot_model.cpp
    src/moveit_collision_checker.cpp)
target_link_libraries(
    moveit_sbpl_planner_plugin 
    ${catkin_LIBRARIES} 
    ${SBPL_LIBRARIES})

install(
    FILES sbpl_interface_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    TARGETS moveit_sbpl_planner_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
