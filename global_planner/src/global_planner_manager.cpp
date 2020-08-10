#include "global_planner/global_planner_manager.h"
#include <nav_msgs/Path.h>

namespace explorer {

GPManager::GPManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  planner_status_ = GPManager::PlannerStatus::NOT_READY;

  global_planner_ = new GlobalPlanner(nh, nh_private);
  full_path_viz_pub = nh_.advertise<nav_msgs::Path>("/glbp/final_global_path", 10);
  global_path_viz_pub = nh_.advertise<nav_msgs::Path>("/glbp/edited_global_path", 10);
  if (!(global_planner_->loadParams())) {
    ROS_ERROR("Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  planner_service_ =
      nh_.advertiseService("global_planner", &GPManager::plannerServiceCallback, this);
  global_planner_service_ = nh_.advertiseService(
      "global_planner/global", &GPManager::globalPlannerServiceCallback, this);
  planner_homing_service_ =
      nh_.advertiseService("global_planner/homing", &GPManager::homingServiceCallback, this);
  planner_set_homing_pos_service_ = nh_.advertiseService(
      "global_planner/set_homing_pos", &GPManager::setHomingPosServiceCallback, this);

  pose_subscriber_ = nh_.subscribe("pose", 100, &GPManager::poseCallback, this);
  odometry_subscriber_ = nh_.subscribe("odometry", 100, &GPManager::odometryCallback, this);
  robot_status_subcriber_ =
      nh_.subscribe("/robot_status", 1, &GPManager::robotStatusCallback, this);
}

bool GPManager::plannerServiceCallback(planner_msgs::planner_srv::Request &req,
                                       planner_msgs::planner_srv::Response &res) {
  global_planner_->setPlanningTriggerTime();
  return true;
}

bool GPManager::homingServiceCallback(planner_msgs::planner_homing::Request &req,
                                      planner_msgs::planner_homing::Response &res) {
  res.path.clear();
  if (getPlannerStatus() == GPManager::PlannerStatus::NOT_READY) {
    ROS_WARN("The planner is not ready.");
    return false;
  }
  std::vector<geometry_msgs::Pose> line_path;
  line_path = global_planner_->getHomingPath(req.header.frame_id);
  if (line_path.size() <= 0) {
    ROS_WARN("Empty Homing Path");
    return true;
  }
  res.path = line_path;
  return true;
}

bool GPManager::globalPlannerServiceCallback(planner_msgs::planner_global::Request &req,
                                             planner_msgs::planner_global::Response &res) {
  res.path.clear();
  if (getPlannerStatus() == GPManager::PlannerStatus::NOT_READY) {
    ROS_WARN("The planner is not ready.");
    return false;
  }
  std::vector<geometry_msgs::Pose> line_path;
  line_path = global_planner_->runGlobalPlanner(req.id);
  if (line_path.size() <= 0) {
    ROS_WARN("Empty Global Path");
    return true;
  }

  res.path = line_path;

  return true;
}

bool GPManager::setHomingPosServiceCallback(
    planner_msgs::planner_set_homing_pos::Request &req,
    planner_msgs::planner_set_homing_pos::Response &res) {
  if (getPlannerStatus() == GPManager::PlannerStatus::NOT_READY) {
    ROS_WARN("The planner is not ready.");
    return false;
  }
  res.success = global_planner_->setHomingPos();
  return true;
}

void GPManager::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose) {
  StateVec state;
  state[0] = pose.pose.pose.position.x;
  state[1] = pose.pose.pose.position.y;
  state[2] = pose.pose.pose.position.z;
  state[3] = tf::getYaw(pose.pose.pose.orientation);
  global_planner_->setState(state);
}

void GPManager::odometryCallback(const nav_msgs::Odometry &odo) {
  StateVec state;
  state[0] = odo.pose.pose.position.x;
  state[1] = odo.pose.pose.position.y;
  state[2] = odo.pose.pose.position.z;
  state[3] = tf::getYaw(odo.pose.pose.orientation);
  global_planner_->setState(state);
}

void GPManager::robotStatusCallback(const planner_msgs::RobotStatus &status) {
  global_planner_->setTimeRemaining(status.time_remaining);
}

GPManager::PlannerStatus GPManager::getPlannerStatus() {
  // if (!ros::ok()) {
  //   ROS_ERROR("ROS node failed.");
  //   return GPManager::PlannerStatus::NOT_READY;
  // }

  // TODO: Should have a list of checking conditions to set the planner as ready.
  // For examples:
  // + ROS ok
  // + All params loaded properly
  // + Map is ready to use
  if (planner_status_ == GPManager::PlannerStatus::READY)
    return GPManager::PlannerStatus::READY;

  return GPManager::PlannerStatus::READY;
}
}