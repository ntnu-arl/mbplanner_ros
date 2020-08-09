cmake_minimum_required(VERSION 2.8.3)
project(global_planner)
# Set OCTOMAP or VOXBLOX
option(USE_OCTOMAP "Use OCTOMAP or VOXBLOX" 0)
if (USE_OCTOMAP)
  add_definitions(-DUSE_OCTOMAP=${USE_OCTOMAP})
endif (USE_OCTOMAP)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)
find_package(planner_common REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${planner_common_INCLUDE_DIRS})
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS})


cs_add_library(
  ${PROJECT_NAME}
  src/global_planner.cpp
  src/global_planner_manager.cpp
  )

cs_add_executable(global_planner_node src/global_planner_ros_node.cpp)
target_link_libraries(global_planner_node ${PROJECT_NAME})
target_link_libraries(global_planner_node ${planner_common_LIBRARIES})

cs_install()
cs_export(LIBRARIES)
