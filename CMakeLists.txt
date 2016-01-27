cmake_minimum_required(VERSION 2.8.3)
project(moveit_planners_sbpl)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(sbpl REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
    eigen_conversions
    leatherman
    moveit_core
    moveit_ros_planning
    roscpp
    sbpl_geometry_utils
    sbpl_manipulation
    pluginlib)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES collision_detection_sbpl moveit_sbpl_planner_plugin
    CATKIN_DEPENDS
        eigen_conversions
        geometric_shapes
        leatherman
        sbpl_manipulation
        roscpp
    DEPENDS)

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    collision_detection_sbpl
    src/collision_plugin_sbpl.cpp
    src/collision_detector_allocator_sbpl.cpp
    src/collision_robot_sbpl.cpp
    src/collision_world_sbpl.cpp)

add_library(
    moveit_sbpl_planner_plugin
    src/sbpl_planner_manager.cpp
    src/sbpl_planning_context.cpp
    src/moveit_robot_model.cpp
    src/moveit_collision_checker.cpp)

target_link_libraries(
    moveit_sbpl_planner_plugin
    ${catkin_LIBRARIES}
    ${SBPL_LIBRARIES}
    collision_detection_sbpl)

install(
    DIRECTORY include/moveit_planners_sbpl
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")

install(
    FILES
        sbpl_interface_plugin_description.xml
        collision_detection_sbpl_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS moveit_sbpl_planner_plugin collision_detection_sbpl
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
