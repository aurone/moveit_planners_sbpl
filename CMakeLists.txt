cmake_minimum_required(VERSION 2.8.3)
project(moveit_planners_sbpl)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(sbpl REQUIRED)
find_package(Boost REQUIRED)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

find_package(catkin REQUIRED
    COMPONENTS
    actionlib
    eigen_conversions
    geometric_shapes
    interactive_markers
    leatherman
    moveit_core
    moveit_msgs
    moveit_ros_planning
    roscpp
    rviz
    smpl
    smpl_ros
    sbpl_collision_checking
    sbpl_pr2_robot_model
    sensor_msgs
    pluginlib
    visualization_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES collision_detection_sbpl moveit_sbpl_planner_plugin
    CATKIN_DEPENDS
        eigen_conversions
        geometric_shapes
        leatherman
        smpl
        smpl_ros
        sbpl_collision_checking
        sbpl_pr2_robot_model
        roscpp
    DEPENDS)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${SBPL_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

qt4_wrap_cpp(
    MOVE_GROUP_COMMAND_PANEL_MOC_SOURCES
    src/joint_variable_command_widget.h
    src/move_group_command_model.h
    src/move_group_command_panel.h
    src/robot_command_model.h)
add_library(
    move_group_command_panel_plugin
    ${MOVE_GROUP_COMMAND_PANEL_MOC_SOURCES}
    src/joint_variable_command_widget.cpp
    src/move_group_command_model.cpp
    src/move_group_command_panel.cpp
    src/robot_command_model.cpp)
target_link_libraries(
    move_group_command_panel_plugin
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES})

add_library(
    collision_detection_sbpl
    src/collision_plugin_sbpl.cpp
    src/collision_common_sbpl.cpp
    src/collision_detector_allocator_sbpl.cpp
    src/collision_robot_sbpl.cpp
    src/collision_world_sbpl.cpp)

target_link_libraries(collision_detection_sbpl ${catkin_LIBRARIES})

add_library(
    moveit_sbpl_planner_plugin
    src/planner_family_manager.cpp
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
        move_group_command_panel_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS
        collision_detection_sbpl
        moveit_sbpl_planner_plugin
        move_group_command_panel_plugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
