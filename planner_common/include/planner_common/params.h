
#ifndef PARAMS_H_
#define PARAMS_H_

#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// Macro for timing: notice that, have to use sequentially;
// otherwise, use different variables
typedef ros::Time TIMER;
#define START_TIMER(x) (x = ros::Time::now())
#define GET_ELAPSED_TIME(x) (float)((ros::Time::now() - x).toSec())

#define ROSPARAM_INFO(msg) std::cout << msg << std::endl

#define ROSPARAM_ERROR(param_name)                                            \
  std::cout << "\033[31m"                                                     \
            << "[ERROR][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\nParam is not set: " << param_name << "\033[0m\n"            \
            << std::endl

#define ROSPARAM_WARN(param_name, default_val)                               \
  std::cout << "\033[33m"                                                    \
            << "[WARN][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\nParam is not set: " << param_name                          \
            << ". Setting to default value: " << default_val << "\033[0m\n"  \
            << std::endl

//
#define PLANNER_INFO(msg) std::cout << msg << std::endl

#define PLANNER_ERROR(msg)                                                    \
  std::cout << "\033[31m"                                                     \
            << "[ERROR][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\n[Planner-Error]: " << msg << "\033[0m\n"                    \
            << std::endl

#define PLANNER_WARN(msg)                                                    \
  std::cout << "\033[33m"                                                    \
            << "[WARN][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\n[Planner-Warn] is not set: " << param_name                 \
            << ". Setting to default value: " << msg << "\033[0m\n"          \
            << std::endl

namespace explorer {

typedef Eigen::Vector4d StateVec;

enum SensorType { kCamera = 0, kLidar = 1 };
struct SensorParamsBase {
  SensorType type;
  double max_range;               // Maximum range for volumetric gain.
  Eigen::Vector3d center_offset;  // Offset from the body center (odometry).
  Eigen::Vector3d rotations;      // Body to sensor; [Y, P, R] (rad).
  Eigen::Vector2d fov;            // [Horizontal, Vertical] angles (rad).
  Eigen::Vector2d resolution;     // Resolution in rad [H x V] for volumetric gain.
  bool loadParams(std::string ns);

  // Check if this state is inside sensor's FOV.
  // pos in (W).
  bool isInsideFOV(StateVec &state, Eigen::Vector3d &pos);
  // Get all endpoints from ray casting models in (W).
  void getFrustumEndpoints(StateVec &state, std::vector<Eigen::Vector3d> &ep);
  // Get all edges in (W).
  void getFrustumEdges(StateVec &state, std::vector<Eigen::Vector3d> &edges);
  // Check if this is potential frontier given sensor FOV.
  // Number of unknow voxels perceived, scaled to map resolution (per meter).
  bool isFrontier(double num_unknown_voxels_normalized);

 private:
  // Rotation matrix
  Eigen::Matrix3d rot_B2S;
  Eigen::Matrix3d rot_S2B;

  // These are to support camera model, approximate as a pyramid.
  // TopLeft, TopRight, BottomRight, BottomLeft.
  Eigen::Matrix<double, 3, 4> edge_points;    // Sensor coordinate, normalized.
  Eigen::Matrix<double, 3, 4> edge_points_B;  // Body coordinate, normalized.
  // These are to support camera model.
  // Place all 4 {top, right, bottom, left} vector into a matrix.
  Eigen::Matrix<double, 3, 4> normal_vectors;

  std::vector<Eigen::Vector3d> frustum_endpoints;    // Sensor coordinate.
  std::vector<Eigen::Vector3d> frustum_endpoints_B;  // Body coordinate.

  double num_voxels_full_fov;
  double frontier_percentage_threshold;
};

struct SensorParams {
  std::vector<std::string> sensor_list;
  std::unordered_map<std::string, SensorParamsBase> sensor;
  bool loadParams(std::string ns);
};

// Set the size of the robot with different protection level based on robot's
// configuration. This could be use in some service calls, so set value
// explicitly to avoid ambiguity.
enum BoundModeType {
  kExtendedBound = 0,  // Use extension to actual size. (default)
  kRelaxedBound = 1,   // Use relaxed extension.
  kMinBound = 2,       // Use minimum bound allowed.
  kExactBound = 3,     // Use its exact size.
  kNoBound = 4,        // Consider a robot as a point.
};
// Represent the robot as a cuboid.
enum RobotType { kAerialRobot = 0, kLeggedRobot };
struct RobotParams {
  // Set the robot type here. For now support multirotor and legged robots.
  RobotType type;
  // Actual size of the robot: length(x) x width(y) x height(z) in meters.
  Eigen::Vector3d size;
  // Minimum extension required for the planner.
  Eigen::Vector3d size_extension_min;
  // Recommend this extension to be used in the planner.
  // size_extension must be at least larger than min_size_extension.
  Eigen::Vector3d size_extension;
  // Offset from the cuboid center to odometry center.
  // Cuboid center = state + center_offset;
  Eigen::Vector3d center_offset;
  // Ratio to compute relaxed extension [0,1].
  //  relax_ratio * size_extension_min + (1-relax_ratio) * size_extension
  double relax_ratio;
  // Bound mode for planning.
  BoundModeType bound_mode;
  // Safety extension
  Eigen::Vector3d safety_extension;

  // Utilities.
  void setBoundMode(BoundModeType bmode);
  // Compute the planning size according to the bound mode setting.
  void getPlanningSize(Eigen::Vector3d &psize);

  bool loadParams(std::string ns);
};

enum BoundedSpaceType { kCuboid = 0, kSphere };
struct BoundedSpaceParams {
  BoundedSpaceType type;
  Eigen::Vector3d min_val;        // [x,y,z] (m): vertex sampling space.
  Eigen::Vector3d max_val;        // [x,y,z] (m)
  Eigen::Vector3d min_extension;  // (min + min_extension): exploration gain.
  Eigen::Vector3d max_extension;  // (max + max_extension): exploration gain.
  Eigen::Vector3d rotations;      // [Y, P, R] wrt W->B coordinate.
  double radius;                  // for Sphere: space for vertex sampling.
  double radius_extension;        // (radius + radius_extension): for
                                  // exploration gain.
  bool loadParams(std::string ns);

  void setCenter(StateVec &state, bool use_extension);
  bool isInsideSpace(Eigen::Vector3d &pos);

 private:
  Eigen::Vector3d root_pos;
  Eigen::Matrix3d rot_B2W;
  Eigen::Vector3d min_val_total;
  Eigen::Vector3d max_val_total;
  double radius_total;
};

// Operation modes depending on the environment.
enum PlanningModeType {
  kBasicExploration = 0,      // Bare-bone functionalities.
  kNarrowEnvExploration = 1,  // Exploration in narrow environment.
};

enum RRModeType {
  kGraph = 0,  // Graph based search (default).
  kTree        // Tree based search,
};

// GBPlanner planning params
struct PlanningParams {
  PlanningModeType type;
  RRModeType rr_mode;
  std::vector<std::string> exp_sensor_list;
  double v_max;
  double v_homing_max;
  double yaw_rate_max;
  // To modify the yaw following the path direction in case of LiDAR.
  bool yaw_tangent_correction;
  // Voxel size to compute exploration gain. Equal to map voxel size or bigger
  // to save computation.
  double exp_gain_voxel_size;
  bool use_ray_model_for_volumetric_gain;
  double free_voxel_gain;
  double occupied_voxel_gain;
  double unknown_voxel_gain;
  double edge_length_min;
  double edge_length_max;
  double edge_overshoot;
  double num_vertices_max;
  double num_edges_max;
  double num_loops_cutoff;
  double num_loops_max;
  double nearest_range;
  double nearest_range_min;
  double nearest_range_max;
  double path_length_penalty;
  double path_direction_penalty;
  double traverse_length_max;
  double traverse_time_max;
  double augment_free_voxels_time;
  double time_budget_limit;
  double overlapped_frontier_radius;
  double sparse_radius;
  bool use_current_state;
  bool augment_free_frustum_en;
  bool adjust_local_sampling_direction;
  bool z_sample_from_ground;
  bool free_frustum_before_planning;
  bool auto_homing_enable;
  bool geofence_checking_enable;
  bool homing_backward;
  bool planning_backward;
  bool safety_aware_enable;
  bool path_safety_enhance_enable;
  std::string global_frame_id;

  bool loadParams(std::string ns);
};

// Parameters for MBPlanner:
struct MBParams {
  // Acceleration limits
  double ax_min;
  double ax_max;
  double ay_min;
  double ay_max;
  double az_min;
  double az_max;
  int iterations, sample_steps;
  double v_max;
  double t, dt, t_min;
  int max_sampling_iters;
  int var_no_of_sampling;
  int later_iter_samples;
  float kdtree_range;
  double safety_distance;
  double min_vertex_separation;
  int obst_steps;
  int branch_limit;
  int use_current_state;
  double extrapolation_dist;
  int yaw_enable;
  int safe_path_samples;
  double exp_dir;
  double min_path_length;
  double forward_angle;
  int x_bias;
  int second_lvl_samples;
  double a_cone;
  int use_angles;
  int clustering;
  double max_traverse_length;
  int min_edge_length;
  double curvature_penalty;
  double safety_gain;
  bool safety_aware_enable;
  double free_voxel_gain;
  double occupied_voxel_gain;
  double unknown_voxel_gain;
  double path_length_penalty;
  double path_direction_penalty;
  double similarity_dist;
  std::vector<std::string> exp_sensor_list;

  bool loadParams(std::string ns);
};

struct RobotDynamicsParams {
  double v_max;
  double v_homing_max;
  double yaw_rate_max;
  bool loadParams(std::string ns);
};

}  // namespace explorer

#endif
