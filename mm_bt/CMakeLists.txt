cmake_minimum_required(VERSION 3.0.2)
project(mm_bt)

## Compile as C++14 (required for raasl_toolbox and modern features)
add_compile_options(-std=c++14)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  marta_msgs
  rami_msgs
  joystick_command
  behaviortree_ros
  raasl_toolbox
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    marta_msgs
    rami_msgs
    joystick_command
    behaviortree_ros
    raasl_toolbox
  DEPENDS 
    EIGEN3
    YAML_CPP
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Zeno Model
add_executable(zeno_model src/zeno_model.cpp)
target_link_libraries(zeno_model ${catkin_LIBRARIES})
add_dependencies(zeno_model 
  ${catkin_EXPORTED_TARGETS}
  joystick_command_generate_messages_cpp
  marta_msgs_generate_messages_cpp
  rami_msgs_generate_messages_cpp
)

## Perceive World (C++ node for buoy detection simulation)
add_executable(perceive_world src/perceive_world.cpp)
target_link_libraries(perceive_world 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(perceive_world 
  ${catkin_EXPORTED_TARGETS}
  marta_msgs_generate_messages_cpp
  rami_msgs_generate_messages_cpp
)

## Mission Visualizer (C++ node for RViz visualization)
add_executable(mission_visualizer src/mission_visualizer.cpp)
target_link_libraries(mission_visualizer 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(mission_visualizer 
  ${catkin_EXPORTED_TARGETS}
  marta_msgs_generate_messages_cpp
  rami_msgs_generate_messages_cpp
)

#############
## Install ##
#############

## Install C++ executables
install(TARGETS 
  zeno_model
  perceive_world
  mission_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/tlm2ros_bridge.py
  src/logs_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Install config files (including YAML and world files)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install rviz files (meshes and .rviz configs)
install(DIRECTORY config/rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE
)

## Install scripts directory (shell scripts only)
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  PATTERN "*.py" EXCLUDE
)

## Install worlds directory (if needed for Stage)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE
)






