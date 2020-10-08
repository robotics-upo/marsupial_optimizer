cmake_minimum_required(VERSION 2.8.3)
project(marsupial_g2o)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib_msgs 
  actionlib
  message_generation
  std_msgs
  marsupial_actions
  upo_actions
  dynamic_reconfigure
)
message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(PCL 1.11 REQUIRED)


# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} )
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} ${PCL_LIBRARIES} )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## C++11 support
add_compile_options(-std=c++14)
## Unfortunately, the 3d-party dependency libg2o requires c++11 starting from ROS Jade.
## Even if the ROS Jade specifications do not want c++11-only packages,
## we cannot compile without c++11 enabled. Another option would be to downgrade  
## libg2o third-party package.
## By now, if you do not want c++11, please refer to the ros indigo version.
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required 
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
endif()
endif()


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   TrajectoryPointMsg.msg
#   TrajectoryMsg.msg
#   FeedbackMsg.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs std_msgs costmap_converter
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   actionlib_msgs
# )

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/OptimizationParams.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES ${EXTERNAL_LIBS}
  CATKIN_DEPENDS
  roscpp
  tf
  std_msgs 
  message_generation 
  message_runtime
  marsupial_actions
  upo_actions
  DEPENDS SUITESPARSE G2O
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  SYSTEM
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Build the teb_local_planner library

add_library(marsupial_g2o 
  # src/marsupial_node.cpp
  src/g2o_edge_distance_vertex.cpp 
  src/g2o_edge_obstacles.cpp
  src/g2o_edge_obstacles_through.cpp
  src/g2o_edge_distance_xyz.cpp
  src/g2o_edge_kinematics.cpp
  src/g2o_edge_equi_distance.cpp
  src/g2o_edge_time.cpp
  src/g2o_edge_velocity.cpp
  src/g2o_edge_acceleration.cpp
  src/g2o_edge_catenary.cpp
  src/optimizer_publisher_tf_node.cpp
  ) 
target_link_libraries(marsupial_g2o
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
)

## Build additional executables

# add_executable(marsupial_node src/marsupial_node.cpp)
# add_dependencies(marsupial_node ${PROJECT_NAME}_gencfg)
# target_link_libraries(marsupial_node marsupial_g2o ${EXTERNAL_LIBS} ${catkin_LIBRARIES})

add_executable(optimizer_local_planner_node src/optimizer_local_planner_node.cpp src/optimizer_local_planner.cpp src/bisection_catenary_3D.cpp)
add_dependencies(optimizer_local_planner_node ${PROJECT_NAME}_gencfg)
target_link_libraries(optimizer_local_planner_node marsupial_g2o ${EXTERNAL_LIBS} ${catkin_LIBRARIES})

add_executable(optimizer_publisher_tf_node src/optimizer_publisher_tf_node.cpp)
add_dependencies(optimizer_publisher_tf_node ${PROJECT_NAME}_gencfg)
target_link_libraries(optimizer_publisher_tf_node marsupial_g2o ${EXTERNAL_LIBS} ${catkin_LIBRARIES})
