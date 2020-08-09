#ifndef GLOBALPLANNERMANAGER_H_
#define GLOBALPLANNERMANAGER_H_
#include <ros/ros.h>

#include "planner_common/graph.h"
#include "planner_common/params.h"
#include "planner_common/trajectory.h"
#include "planner_msgs/RobotStatus.h"
#include "planner_msgs/pci_trigger.h"
#include "planner_msgs/planner_global.h"
#include "planner_msgs/planner_homing.h"
#include "planner_msgs/planner_request_path.h"
#include "planner_msgs/planner_set_global_bound.h"
#include "planner_msgs/planner_set_homing_pos.h"
#include "planner_msgs/planner_srv.h"

#include "global_planner/global_planner.h"
#include "planner_common/visualizer.h"

// #include <gbplanner_trajopt/gbplanner_trajopt.h>

namespace explorer {

class GPManager {
 public:
  enum PlannerStatus { NOT_READY = 0, READY };
  enum PlanningState { kNull = 0, kStart, kBoxClearance, kExploration, kStop };
  GPManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer planner_service_;
  ros::ServiceServer global_planner_service_;
  ros::ServiceServer planner_homing_service_;
  ros::ServiceServer planner_set_homing_pos_service_;
  ros::ServiceServer planner_geofence_service_;
  ros::ServiceServer planner_set_global_bound_service_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber robot_status_subcriber_;
  ros::ServiceClient map_save_service_;

  ros::Publisher full_path_viz_pub, global_path_viz_pub;
  ros::Publisher trajectory_pub_;

  GlobalPlanner* global_planner_;
  // GbplannerTrajOpt* traj_opt_;

  PlannerStatus planner_status_;
  PlanningState planning_state_;

  bool plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res);
  bool homingServiceCallback(planner_msgs::planner_homing::Request& req,
                             planner_msgs::planner_homing::Response& res);
  bool setHomingPosServiceCallback(planner_msgs::planner_set_homing_pos::Request& req,
                                   planner_msgs::planner_set_homing_pos::Response& res);

  bool globalPlannerServiceCallback(planner_msgs::planner_global::Request& req,
                                    planner_msgs::planner_global::Response& res);

  void untraversablePolygonCallback(const geometry_msgs::PolygonStamped& polygon_msgs);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void odometryCallback(const nav_msgs::Odometry& odo);
  void robotStatusCallback(const planner_msgs::RobotStatus& status);

  GPManager::PlannerStatus getPlannerStatus();
};
}

#endif