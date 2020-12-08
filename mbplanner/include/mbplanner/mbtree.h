

#ifndef mbTREE_H_
#define mbTREE_H_

#include <kdtree/kdtree.h>
#include <time.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "planner_common/graph_base.h"
#include "planner_common/map_manager.h"
#include "planner_common/params.h"
#include "planner_common/trajectory.h"
#include "planner_common/visualizer.h"
#include "planner_msgs/PlannerLogger.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#ifdef USE_OCTOMAP
#include "planner_common/map_manager_octomap_impl.h"
#else
#include "planner_common/map_manager_voxblox_impl.h"
#endif

#define START_TIMER(x) (x = ros::Time::now())
#define GET_ELAPSED_TIME(x) (float)((ros::Time::now() - x).toSec())
#define GET_ELAPSED_TIME_MILLI(x) (double)((ros::Time::now() - x).toNSec())

namespace explorer {
namespace mbplanner {

class MBTree {
 public:
  MBTree(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~MBTree();
  void reset();  // Empty state_tree
  void deleteTree(StateNode*);
  int buildTree(bool, bool,
                bool);  // Build the tree. Returns the number of vertices in the tree
  // void visualize_tree();  // Visualize tree in Rviz. Takes vector of edges
  void set_state(StateNode);      // Update the current position of the robot in the
                                  // MBTree object
  bool evaluateGraph();           // Find the exploration gains.
  void computeExplorationGain();  // Compute volumetric gain part
  VolumetricGain computeIndividualExplorationGain(StateNode& leaf, bool vis_en);
  void computeVolumetricGainRayModel(
      StateVec& state, VolumetricGain& vgain,
      bool);  // Volumetric gain calculation using raycasting model
  std::vector<StateNode*> findLeafVertices();
  std::vector<StateNode*> getShortestPath(StateNode*, bool);
  void timerCallback(const ros::TimerEvent& event);
  std::vector<geometry_msgs::Pose> getBestPath(int& status);

  void printTimes();
  int predict_hit(StateNode* leaf);
  bool compareAngles(double, double, double);
  void computeDistanceMap(Eigen::Vector3d& map_center, Eigen::Vector3d& map_size);
  bool is_line_good(StateNode*, StateNode*);
  void setRoot(StateNode);
  std::vector<StateNode*> get_prev_path();
  void setPrevPath(std::vector<StateNode*>);
  bool inside_global_box(Eigen::Vector3d);
  bool inside_local_box(Eigen::Vector3d);
  std::vector<Eigen::Vector3d> getEigenPath(std::vector<StateNode*>);
  std::vector<geometry_msgs::Pose> getPosePath(std::vector<StateNode*>);
  std::vector<geometry_msgs::Pose> getSafetyPath();
  Eigen::Vector3d estimateDirectionFromPath(std::vector<Eigen::Vector3d>);
  Eigen::Vector3d getNextVel();
  void resetExpDir();
  void setGivenVel(Eigen::Vector3d);
  void setSafetyPath(std::vector<StateNode*>);
  void setSafetyPath(std::vector<geometry_msgs::Pose>);
  double calculate_curvature(std::vector<StateNode*>);
  void getFreeSpacePointCloud(std::string, StateVec,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void freePointCloudtimerCallback(const ros::TimerEvent& event);

  void clusterPaths(std::vector<StateNode*>& clustered_leaves, double dist_threshold = 1.0);
  void getGlobalGraphPaths(std::vector<SerializeVertex>& vertices,
                           std::vector<StateNode*> clustered_leaves);
  SerializeVertex getSerVert(StateNode*);
  void serializeAndPublish(std::vector<SerializeVertex>);
  void publishLogInfo();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Parameters:
  std::string ns;
  MBParams mb_params_;
  RobotParams robot_params_;
  std::string world_frame_id;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams global_space_params_;
  SensorParams sensor_params_;
  SensorParams free_frustum_params_;

  std::fstream csv_file;

#ifdef USE_OCTOMAP
  explorer::MapManagerOctomap* map_manager_;
#else
  explorer::MapManagerVoxblox<explorer::MapManagerVoxbloxServer,
                              explorer::MapManagerVoxbloxVoxel>* map_manager_;
#endif

  // Viz
  Visualization* visualizer_;

  ros::Publisher planner_logger_pub_;
  ros::Publisher tree_pub_;
  ros::Publisher free_cloud_pub_;
  const double robot_lifetime = 0;

  std::mt19937 gen_;
  StateNode* tree_root_;
  std::vector<StateNode*> leaf_vertices_;
  std::vector<StateNode> viz_state_tree_;
  StateNode current_state_;
  StateNode* next_root_;
  StateNode root_storage;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> obs_pcl_;
  std::vector<StateNode*> safe_path_;
  Eigen::Vector3d given_vel_;

  bool odometry_ready;
  std::queue<StateNode> robot_state_queue_;
  double exploring_direction_;
  StateNode* best_vertex_;
  std::vector<StateNode*> final_path_;
  std::vector<StateNode*> full_path_;
  std::vector<StateNode*> prev_path_;
  std::vector<geometry_msgs::Pose> actual_best_path;
  std::vector<geometry_msgs::Pose> orig_path_;
  kdtree* kd_tree_;
  kdtree* main_tree_;
  Eigen::Vector3d next_vels_;

  const double kTimerPeriod = 0.1;
  ros::Timer periodic_timer_;
  ros::Timer periodic_timer_free_pcl_;
  ros::Timer vol_logger_;
  ros::Time rostime_start_;
  ros::Time ttime;
  int robot_history_count_;
  int backward_path_counter_;
  double total_path_length_;
  double num_occupied_voxels_;
  double num_free_voxels_;
  bool first_odom;

  // Logging params:
  std::shared_ptr<SampleStatistic> stat_;
  // Path length
  double path_length_;

  std::shared_ptr<std::uniform_real_distribution<double>>
      distribution_x_;  //(mb_params_.ax_min, mb_params_.ax_max);
  std::shared_ptr<std::uniform_real_distribution<double>>
      distribution_y_;  //(mb_params_.ay_min, mb_params_.ay_max);
  std::shared_ptr<std::uniform_real_distribution<double>>
      distribution_z_;  //(mb_params_.az_min, mb_params_.az_max);
  std::shared_ptr<std::uniform_real_distribution<double>>
      distribution_dir_;  //(mb_params_.az_min, mb_params_.az_max);
  std::shared_ptr<std::uniform_real_distribution<double>>
      distribution_mag_;  //(mb_params_.az_min, mb_params_.az_max);

  inline void truncateYaw(double& x) {
    if (x > M_PI)
      x -= 2 * M_PI;
    else if (x < -M_PI)
      x += 2 * M_PI;
  }
};

}  // mbplanner
}  // explorer
#endif