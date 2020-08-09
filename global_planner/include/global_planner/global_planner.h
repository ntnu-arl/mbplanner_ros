#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <kdtree/kdtree.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/map_manager.h"
#include "planner_common/params.h"
#include "planner_common/random_sampler.h"
#include "planner_common/trajectory.h"
#ifdef USE_OCTOMAP
#include "planner_common/map_manager_octomap_impl.h"
#else
#include "planner_common/map_manager_voxblox_impl.h"
#endif

#include "planner_common/visualizer.h"

// #include <gbplanner_trajopt/gbplanner_trajopt.h>

// Publish all gbplanner rviz topics or not.
#define FULL_PLANNER_VIZ 1

namespace explorer {

class RobotStateHistory {
 public:
  RobotStateHistory();
  void addState(StateVec* s);
  bool getNearestState(const StateVec* state, StateVec** s_res);
  bool getNearestStateInRange(const StateVec* state, double range, StateVec** s_res);
  bool getNearestStates(const StateVec* state, double range, std::vector<StateVec*>* s_res);
  void reset();
  std::vector<StateVec*> state_hist_;

 private:
  // Kd-tree for nearest neigbor lookup.
  kdtree* kd_tree_;
};

class GlobalPlanner {
 public:
  // Graph status:
  enum GraphStatus {
    OK = 0,                // Everything is OK as expected.
    ERR_KDTREE,            // Could not search nearest neigbors from kdtree.
    ERR_NO_FEASIBLE_PATH,  // Could not find any path.
    NO_GAIN,               // No non-zero gain found.
    NOT_OK,                // Any other errors.
  };

  /* Constructor */
  GlobalPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  std::string world_frame_id;

  /* Parameters */
  bool loadParams();
  void initializeParams();

  /* Path Improvement */
  bool modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl, Eigen::Vector3d& p0,
                  Eigen::Vector3d& p1, Eigen::Vector3d& p1_mod);
  bool improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                       std::vector<geometry_msgs::Pose>& path_mod);

  /* Build Graph */
  // Check collision free and extend graph with this new vertex.
  void expandGraph(std::shared_ptr<GraphManager> graph_manager, StateVec& new_state,
                   ExpandGraphReport& rep, bool allow_short_edge = false);
  // Add edges only from this vertex.
  void expandGraphEdges(std::shared_ptr<GraphManager> graph_manager, Vertex* new_vertex,
                        ExpandGraphReport& rep);

  void computeVolumetricGain(StateVec& state, VolumetricGain& vgain, bool vis_en = false);
  void computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                     bool vis_en = false);
  void computeVolumetricGainRayModelNoBound(StateVec& state, VolumetricGain& vgain);

  /* Global planning */
  // Homing:
  std::vector<geometry_msgs::Pose> searchHomingPath(std::string tgt_frame,
                                                    const StateVec& cur_state);
  std::vector<geometry_msgs::Pose> getHomingPath(std::string tgt_frame);
  bool setHomingPos();
  // Global planner trigger
  std::vector<geometry_msgs::Pose> runGlobalPlanner(int vertex_id = 0);

  MapManager::VoxelStatus getPathStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      bool stop_at_unknown_voxel) const;  // For others to use

  MapManager::VoxelStatus getBoxStatus(const Eigen::Vector3d& p,
                                       bool stop_at_unknown_voxel) const;

  void setState(StateVec& state);
  void setPlanningTriggerTime();
  void setTimeRemaining(double t) { current_battery_time_remaining_ = t; }

  void localTreeCallback(const std_msgs::UInt8MultiArray& msg);

  std::vector<int> performShortestPathsClustering(
      const std::shared_ptr<GraphManager> graph_manager, const ShortestPathsReport& graph_rep,
      std::vector<Vertex*>& vertices, double dist_threshold = 1.0,
      double principle_path_min_length = 1.0, bool refinement_enable = true);

 private:
  // ROS Node handles:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber local_tree_subscriber_;

  std::shared_ptr<GraphManager> global_graph_;
  std::shared_ptr<GraphManager> local_graph_;
  ShortestPathsReport global_graph_rep_;  // shortest path to source 0
  ShortestPathsReport local_graph_rep_;   // shortest path to source 0

  // Precompute params for planner.
  Eigen::Vector3d robot_box_size_;
  int planning_num_vertices_max_;
  int planning_num_edges_max_;

  // Current state of the robot, updated from odometry.
  StateVec current_state_;
  StateVec state_for_planning_;
  StateVec last_state_marker_;
  bool odometry_ready;

  int planner_trigger_time_;
  const int backtracking_queue_max_size = 500;
  std::queue<StateVec> robot_backtracking_queue_;
  Vertex* robot_backtracking_prev_;

  // Time keeper
  std::shared_ptr<SampleStatistic> stat_;
  // Temprary variable for timing purpose.
  ros::Time ttime;

  // For visualization.
  Visualization* visualization_;

// Map manager:
#ifdef USE_OCTOMAP
  MapManagerOctomap* map_manager_;
#else
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>* map_manager_;
#endif

  ros::Time rostime_start_;
  double current_battery_time_remaining_;

  // Params required for planning.
  SensorParams sensor_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams global_space_params_;
  PlanningParams planning_params_;
  RandomSampler random_sampler_;            // x,y,z,yaw: for exploration purpose
  RobotDynamicsParams robot_dynamics_params_;
  BoundingBoxType global_bound_;

  // Robot state history
  std::shared_ptr<RobotStateHistory> robot_state_hist_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> obs_pcl_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> feasible_corridor_pcl_;

  bool sampleRandomState(StateVec& state);
  // bool sampleVertex(StateVec& state);
  bool sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                    StateVec& sampled_state);

  // Add a collision-free path to the graph.
  bool addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                         const std::vector<Vertex*>& vertices);

  bool addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                         const std::vector<geometry_msgs::Pose>& path);

  const double kGlobalGraphUpdateTimerPeriod = 0.7;
  const double kGlobalGraphUpdateTimeBudget = 0.5;
  ros::Timer global_graph_update_timer_;
  void expandGlobalGraphTimerCallback(const ros::TimerEvent& event);
  ros::Timer periodic_timer_;
  void timerCallback(const ros::TimerEvent& event);

  bool comparePathWithDirectionApprioximately(const std::vector<geometry_msgs::Pose>& path,
                                              double yaw);
  bool isPathInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end) const;
  bool isCollision(const Eigen::Vector3d& p) const;

  // Other required functions:
  inline void truncateYaw(double& x) {
    if (x > M_PI)
      x -= 2 * M_PI;
    else if (x < -M_PI)
      x += 2 * M_PI;
  }

  inline void offsetZAxis(StateVec& state, bool down = false) {
    if (robot_params_.type == RobotType::kLeggedRobot) {
      if (down)
        state[2] -= random_sampler_.getZOffset();
      else
        state[2] += random_sampler_.getZOffset();
    }
  }
};

}  // namespace explorer

#endif