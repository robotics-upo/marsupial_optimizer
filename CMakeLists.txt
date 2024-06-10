# Ceres Solver - A fast non-linear least squares minimizer
# Copyright 2015 Google Inc. All rights reserved.
# http://ceres-solver.org/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Google Inc. nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: keir@google.com (Keir Mierle)
# Only Ceres itself should be compiled with CERES_BUILDING_SHARED_LIBRARY
# defined, any users of Ceres will have CERES_USING_SHARED_LIBRARY defined
# for them in Ceres' config.h if appropriate.

cmake_minimum_required(VERSION 2.8.3)
project(marsupial_optimizer CXX)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib_msgs 
  actionlib
  message_generation
  std_msgs
  trajectory_msgs
  upo_actions
  catenary_checker
  dynamic_reconfigure
  octomap_msgs
  octomap_ros
)

find_package(PCL 1.8 REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS
  Charts
  Widgets
)

find_package(Qt5Charts REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  marsupial_trajectory_optimized.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  std_msgs
  trajectory_msgs
)

generate_dynamic_reconfigure_options(
  cfg/OptimizationParams.cfg
)

catkin_package(
  INCLUDE_DIRS include 
  ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES ${EXTERNAL_LIBS} yaml-cpp
  CATKIN_DEPENDS
  roscpp
  tf
  std_msgs 
  geometry_msgs
  trajectory_msgs
  message_generation 
  message_runtime
  upo_actions
  catenary_checker
  pcl_conversions 
  pcl_ros
  octomap_msgs 
  octomap_ros
  DEPENDS system_lib octomap_msgs octomap_ros yaml-cpp
)

# message(STATUS "OPTIMIZER_PRINT include dir: ${catkin_INCLUDE_DIRS}")

include_directories(
  include/ 
  ${catkin_INCLUDE_DIRS}  
  ${PCL_INCLUDE_DIRS} 
  ${CERES_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
  ${Qt5Charts_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${PCL_LIBRARIES} )

if (BUILD_SHARED_LIBS)
  remove_definitions(-DCERES_BUILDING_SHARED_LIBRARY)
endif()

# add_library(marsupial_optimizer 
# )

# target_link_libraries(marsupial_optimizer
#    ${EXTERNAL_LIBS}
#    ${catkin_LIBRARIES}
#    yaml-cpp
# )

# add_executable(optimizer_local_planner_fix_ugv src/optimizer_local_planner_fix_ugv.cc)
# target_link_libraries(optimizer_local_planner_fix_ugv Ceres::ceres ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${EXTERNAL_LIBS})

add_executable(optimizer_local_planner_fix_ugv_node src/optimizer_local_planner_fix_ugv_node.cpp src/optimizer_local_planner_fix_ugv.cc src/marker_publisher.cpp )
add_dependencies(optimizer_local_planner_fix_ugv_node ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS} )
target_link_libraries(optimizer_local_planner_fix_ugv_node  catenary_checker Ceres::ceres yaml-cpp ${catkin_LIBRARIES} ${CERES_LIBRARIES} )

add_executable(optimizer_local_planner_node src/optimizer_local_planner_node.cpp src/optimizer_local_planner.cc src/marker_publisher.cpp src/manage_computed_path.cpp)
add_dependencies(optimizer_local_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS} )
target_link_libraries(optimizer_local_planner_node catenary_checker Ceres::ceres ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp Qt5::Charts
Qt5::Widgets dbscan_line)

add_executable(optimizer_publisher_tf_node src/optimizer_publisher_tf_node.cpp src/marker_publisher.cpp) 
add_dependencies(optimizer_publisher_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS} )
target_link_libraries(optimizer_publisher_tf_node catenary_checker Ceres::ceres ${catkin_LIBRARIES} ${CERES_LIBRARIES} Qt5::Charts
Qt5::Widgets dbscan_line)

# add_executable(test_catenary_optimizer src/test_catenary_optimizer.cc src/marker_publisher.cpp)
# add_dependencies(test_catenary_optimizer  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# target_link_libraries(test_catenary_optimizer catenary_checker Ceres::ceres ${catkin_LIBRARIES} ${CERES_LIBRARIES} Qt5::Charts
# Qt5::Widgets dbscan_line)

# add_executable(test_node_yaml src/test_node_yaml.cpp) 
# add_dependencies(test_node_yaml  ${PROJECT_NAME}_gencfg)
# target_link_libraries(test_node_yaml marsupial_optimizer ${EXTERNAL_LIBS} ${catkin_LIBRARIES} yaml-cpp)

# add_executable(catenary_solver_test_ceres src/catenary_solver_test_ceres.cpp )
# # add_dependencies(optimizer_local_planner_fix_ugv_node ${PROJECT_NAME}_gencfg)
# target_link_libraries(catenary_solver_test_ceres marsupial_optimizer  Ceres::ceres ${catkin_LIBRARIES} ${EXTERNAL_LIBS} ${CERES_LIBRARIES})

# add_executable(local_plann_optimizer_ceres_test src/local_plann_optimizer_ceres_test.cpp )
# target_link_libraries(local_plann_optimizer_ceres_test marsupial_optimizer  Ceres::ceres ${catkin_LIBRARIES} ${EXTERNAL_LIBS} ${CERES_LIBRARIES})

# As this is a C file #including <math.h> we have to explicitly add the math
# library (libm). Although some compilers (dependent upon options) will accept
# the indirect link to libm via Ceres, at least GCC 4.8 on pure Debian won't.
if (NOT MSVC)
endif (NOT MSVC)

if (GFLAGS)
  
endif (GFLAGS)
