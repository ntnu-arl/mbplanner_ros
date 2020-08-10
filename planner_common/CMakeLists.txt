cmake_minimum_required(VERSION 2.8.3)
project(planner_common)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roscpp_serialization REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/graph_manager.cpp
  src/graph.cpp
  src/params.cpp
  src/trajectory.cpp
  src/visualizer.cpp
  src/random_sampler.cpp
  src/map_manager_octomap_impl.cpp
  src/map_manager_voxblox_impl.cpp
)

cs_install()
cs_export(LIBRARIES)