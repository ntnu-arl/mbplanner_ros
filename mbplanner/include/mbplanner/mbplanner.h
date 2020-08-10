#ifndef MBPLANNER_H_
#define MBPLANNER_H_

#include <ros/ros.h>

#include "mbplanner/mbtree.h"
#include "planner_common/params.h"
#include "planner_msgs/RobotStatus.h"
#include "planner_msgs/planner_set_vel.h"
#include "planner_msgs/planner_srv.h"

namespace explorer {

class MBPlanner {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer planner_service_;
  ros::ServiceServer planner_set_vel_service_;

  ros::Subscriber odometry_subscriber_;

  mbplanner::MBTree* mbtree_;

  StateNode current_state_;
  geometry_msgs::Pose current_pose;

  std::string ns;
  MBParams mb_params_;

  bool first_plan;
  bool do_set_vel;
  bool planning_from_prev_node;

  Eigen::Vector3d vel_to_set;

  bool plannerServiceCallback(
      planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);  // To call the planner
  void odometryCallback(const nav_msgs::Odometry& odo);

  bool setVelCb(planner_msgs::planner_set_vel::Request& req,
                planner_msgs::planner_set_vel::Response& res);

 public:
  MBPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~MBPlanner();
};

}  // explorer
#endif