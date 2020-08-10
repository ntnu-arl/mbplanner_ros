#ifndef GBPLANNER_RVIZ_H_
#define GBPLANNER_RVIZ_H_

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <queue>

#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/map_manager.h"
#include "planner_common/params.h"
#include "planner_common/trajectory.h"

#include "planner_common/random_sampler.h"

namespace explorer {

class Visualization {
 public:
  Visualization(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  // Visualize planning workspace including global and local bounds.
  void visualizeWorkspace(StateVec &state, BoundedSpaceParams &global_ws,
                          BoundedSpaceParams &local_ws);
  // Visualize executing path.
  void visualizeRefPath(const std::vector<geometry_msgs::Pose> &path);
  // Visualize volumetric gain.
  void visualizeVolumetricGain(
      Eigen::Vector3d &bound_min, Eigen::Vector3d &bound_max,
      std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> &voxels,
      double voxel_size);
  // Visualize rays from a state
  void visualizeRays(const StateVec state, const std::vector<Eigen::Vector3d> ray_endpoints);
  // Visualize global graph.
  void visualizeGlobalGraph(const std::shared_ptr<GraphManager> graph_manager);
  void visualizeHomingPaths(const std::shared_ptr<GraphManager> graph_manager,
                            const ShortestPathsReport &graph_rep, int current_id);

  void visualizeGlobalPaths(const std::shared_ptr<GraphManager> graph_manager,
                            std::vector<int> &to_frontier_ids, std::vector<int> &to_home_ids);

  void visualizePCL(const pcl::PointCloud<pcl::PointXYZ> *pcl);
  void visualizeHyperplanes(Eigen::Vector3d &center,
                            std::vector<Eigen::Vector3d> &hyperplane_list,
                            std::vector<Eigen::Vector3d> &tangent_point_list);
  void visualizeModPath(const std::vector<geometry_msgs::Pose> &path);

  void visualizeMBPath(const std::vector<geometry_msgs::Pose> &final_path,
                       const std::vector<geometry_msgs::Pose> &full_path,
                       const std::vector<geometry_msgs::Pose> &original_path);
  void visualizeMBTree(StateNode *root);
  void visualizeMBTreeEvaluation(std::vector<StateNode *> &leaf_vertices);
  void visualizeSimilarPaths(
      const std::vector<std::vector<geometry_msgs::Pose>> &valid_paths,
      const std::vector<std::vector<geometry_msgs::Pose>> &invalid_paths);
  void visualizeRobotState(const StateNode &state, const Eigen::Vector3d size,
                           const double &direction);
  void visualizeSensorFOV(StateVec &state, SensorParams &sensor_params);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // General
  ros::Publisher planning_workspace_pub_;
  ros::Publisher sensor_fov_pub_;
  // Global Planner
  ros::Publisher planning_global_graph_pub_;
  ros::Publisher planning_homing_pub_;
  ros::Publisher planning_global_path_pub_;
  ros::Publisher ref_paths_pub_;
  ros::Publisher mod_path_pub_;
  ros::Publisher rays_pub_;
  ros::Publisher hyperplanes_pub_;
  ros::Publisher volumetric_gain_pub_;
  ros::Publisher pcl_pub_;
  // MBPlanner:
  ros::Publisher viz_final_path_pub_;
  ros::Publisher viz_full_path_pub_;
  ros::Publisher viz_original_path_pub_;
  ros::Publisher viz_tree_pub_;
  ros::Publisher safety_paths_pub_;
  ros::Publisher viz_gains_pub_;
  ros::Publisher similar_paths_pub_;
  ros::Publisher robot_box_pub_;
  ros::Publisher exploration_direction_pub_;

  std::string world_frame_id = "world";
  const double ws_lifetime = 0;  // infinite
  const double graph_lifetime = 0.0;
  const double shortest_paths_lifetime = 0.0;
  const double robot_lifetime = 0;
  const double sampler_lifetime = 0;
  const double ray_lifetime = 0;

  int best_path_id_;
  bool getHeatMapColor(float value, float &red, float &green, float &blue);
};

}  // namespace explorer

#endif
