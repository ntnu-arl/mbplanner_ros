/* This file has the definition for the callback to the PCI trigger service */

#include "mbplanner/mbplanner.h"
#include <nav_msgs/Path.h>

namespace explorer {

MBPlanner::~MBPlanner() {}

MBPlanner::MBPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  mbtree_ = new mbplanner::MBTree(nh, nh_private);

  planner_service_ =
      nh_.advertiseService("mbplanner", &MBPlanner::plannerServiceCallback, this);
  planner_set_vel_service_ =
      nh_.advertiseService("planner_set_vel", &MBPlanner::setVelCb, this);

  odometry_subscriber_ = nh_.subscribe("odometry", 10, &MBPlanner::odometryCallback, this);

  this->planning_from_prev_node = false;
  this->first_plan = true;
  this->do_set_vel = false;
}

bool MBPlanner::setVelCb(planner_msgs::planner_set_vel::Request& req,
                         planner_msgs::planner_set_vel::Response& res) {
  do_set_vel = req.set;
  vel_to_set(0) = req.root_vel.linear.x;
  vel_to_set(1) = req.root_vel.linear.y;
  vel_to_set(2) = req.root_vel.linear.z;

  res.success = true;
  return true;
}

bool MBPlanner::plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                                       planner_msgs::planner_srv::Response& res) {
  if (first_plan) {
    mbtree_->resetExpDir();
    first_plan = false;
  }
  res.path.clear();
  mbtree_->reset();
  int nodes;
  geometry_msgs::Pose input_root = req.root_pose;
  // Logic to set the root vertex for tree generation
  if (planning_from_prev_node == false) {
    if (input_root.position.x == 0.0 && input_root.position.y == 0.0 &&
        input_root.position.z == 0.0) {
      ROS_INFO("MBPLANNER.CPP: Planning from current state.");
      if (do_set_vel) {
        mbtree_->setGivenVel(vel_to_set);
        nodes = mbtree_->buildTree(false, true, true);
      } else {
        mbtree_->setGivenVel(Eigen::Vector3d::Zero());
        nodes = mbtree_->buildTree(false, true, false);
      }
    } else {
      StateNode new_root;
      std::vector<StateNode*> prev_path = mbtree_->get_prev_path();
      new_root.position(0) = input_root.position.x;
      new_root.position(1) = input_root.position.y;
      new_root.position(2) = input_root.position.z;
      new_root.velocities = mbtree_->getNextVel();
      mbtree_->setRoot(new_root);

      if (do_set_vel) {
        mbtree_->setGivenVel(vel_to_set);
        nodes = mbtree_->buildTree(false, false, true);
      } else {
        mbtree_->setGivenVel(Eigen::Vector3d::Zero());
        nodes = mbtree_->buildTree(false, false, false);
      }
    }
  } else {
    if (do_set_vel) {
      mbtree_->setGivenVel(vel_to_set);
      nodes = mbtree_->buildTree(false, true, true);
    } else
      nodes = mbtree_->buildTree(false, true, false);
    planning_from_prev_node = false;
  }

  int path_status;

  ROS_INFO("Tree built with [%d] nodes", nodes);
  bool found = false;
  do_set_vel = false;

  /* Go to safety path if no tree built */
  if (nodes < 2) {
    ROS_WARN("Unable to span the tree");
    std::vector<StateNode*> new_prev_path;
    res.path = mbtree_->getSafetyPath();
    ROS_INFO("Safety path found: ");
    for (int i = 0; i < res.path.size(); i++) {
      std::cout << res.path[i].position.x << ", " << res.path[i].position.y << ", "
                << res.path[i].position.z << std::endl;
    }
    StateNode* req_pt;
    std::vector<StateNode*> prev_path = mbtree_->get_prev_path();
    // for(auto p:prev_path) p->printit();
    int req_pt_index = 0;
    if (prev_path.size() > 0) {
      for (int s = 1; s < prev_path.size(); s++) {
        if (prev_path[s]->special_point)
        // if((prev_path[s].position - current_state_.position).norm() < 0.4)
        {
          req_pt = prev_path[s];
          found = true;
          req_pt_index = s;
          break;
        }
      }
      // Checking if the req_pt is not to far:
      Eigen::Vector3d first_pt(res.path[0].position.x, res.path[0].position.y,
                               res.path[0].position.z);
      if ((first_pt - current_state_.position).norm() > 2.0) {
        std::vector<geometry_msgs::Pose> curr_state_path;
        ROS_WARN("Path too far");
        geometry_msgs::Pose p;
        p.position.x = current_state_.position(0);
        p.position.y = current_state_.position(1);
        p.position.z = current_state_.position(2);
        if (mb_params_.yaw_enable) {
          p.orientation.x = current_pose.orientation.x;
          p.orientation.y = current_pose.orientation.y;
          p.orientation.z = current_pose.orientation.z;
          p.orientation.w = current_pose.orientation.w;
        } else {
          p.orientation.x = 0.0;
          p.orientation.y = 0.0;
          p.orientation.z = 0.0;
          p.orientation.w = 1.0;
        }
        curr_state_path.push_back(p);
        res.path = curr_state_path;
        // new_prev_path.push_back(prev_path[req_pt_index]);
        StateNode* curr_state_pt = new StateNode;
        curr_state_pt->position = current_state_.position;
        curr_state_pt->velocities = current_state_.velocities;
        curr_state_pt->special_point = true;
        new_prev_path.push_back(curr_state_pt);
        mbtree_->setSafetyPath(new_prev_path);
        mbtree_->setPrevPath(new_prev_path);
        return true;
      } else {
        planning_from_prev_node = true;
        std::vector<geometry_msgs::Pose> new_safety_path = res.path;
        for (int s = 0; s < req_pt_index; s++) {
          geometry_msgs::Pose p;
          p.position.x = prev_path[s]->position(0);
          p.position.y = prev_path[s]->position(1);
          p.position.z = prev_path[s]->position(2);
          new_safety_path.push_back(p);
        }
        mbtree_->setSafetyPath(new_safety_path);
        if (prev_path.size() > req_pt_index + 1) {
          for (int s = req_pt_index; s < prev_path.size(); s++) {
            new_prev_path.push_back(prev_path[s]);
          }
        } else {
          new_prev_path.push_back(prev_path[req_pt_index]);
          StateNode* curr_state_pt = new StateNode;
          curr_state_pt->position = current_state_.position;
          curr_state_pt->velocities = current_state_.velocities;
          new_prev_path.push_back(curr_state_pt);
        }
        return true;
      }
    } else
      return false;
  }

  /* Evaluate graph */
  bool status = mbtree_->evaluateGraph();
  /* Go to safety path if zero gain for all paths*/
  if (status == true) {
    std::vector<geometry_msgs::Pose> path = mbtree_->getBestPath(res.status);
    int count = 0;
    for (geometry_msgs::Pose p : path) {
      count++;
      if (count == path.size()) {
        if ((Eigen::Vector3d(p.position.x, p.position.y, p.position.z) -
             Eigen::Vector3d::Zero())
                .norm() < 0.001) {
          path.erase(path.end());
        }
      }
    }

    res.path = path;
    mbtree_->printTimes();
    mbtree_->publishLogInfo();
    ROS_INFO("LOCAL PLANNER SUCCESSFULLY FOUND PATH");
  } else {
    ROS_WARN("Zero gain");
    std::vector<StateNode*> new_prev_path;
    res.path = mbtree_->getSafetyPath();
    StateNode* req_pt;
    std::vector<StateNode*> prev_path = mbtree_->get_prev_path();
    // for(auto p:prev_path) p->printit();
    int req_pt_index = 0;
    if (prev_path.size() > 0) {
      for (int s = 1; s < prev_path.size(); s++) {
        if (prev_path[s]->special_point)
        // if((prev_path[s].position - current_state_.position).norm() < 0.4)
        {
          req_pt = prev_path[s];
          found = true;
          req_pt_index = s;
          break;
        }
      }
      // Checking if the req_pt is not to far:
      Eigen::Vector3d first_pt(res.path[0].position.x, res.path[0].position.y,
                               res.path[0].position.z);
      if ((first_pt - current_state_.position).norm() > 2.0) {
        std::vector<geometry_msgs::Pose> curr_state_path;
        std::cout << "Path too far" << std::endl;
        geometry_msgs::Pose p;
        p.position.x = current_state_.position(0);
        p.position.y = current_state_.position(1);
        p.position.z = current_state_.position(2);
        if (mb_params_.yaw_enable) {
          p.orientation.x = current_pose.orientation.x;
          p.orientation.y = current_pose.orientation.y;
          p.orientation.z = current_pose.orientation.z;
          p.orientation.w = current_pose.orientation.w;
        } else {
          p.orientation.x = 0.0;
          p.orientation.y = 0.0;
          p.orientation.z = 0.0;
          p.orientation.w = 1.0;
        }
        curr_state_path.push_back(p);
        res.path = curr_state_path;
        // new_prev_path.push_back(prev_path[req_pt_index]);
        StateNode* curr_state_pt = new StateNode;
        curr_state_pt->position = current_state_.position;
        curr_state_pt->velocities = current_state_.velocities;
        curr_state_pt->special_point = true;
        new_prev_path.push_back(curr_state_pt);
        mbtree_->setSafetyPath(new_prev_path);
        mbtree_->setPrevPath(new_prev_path);
        return true;
      } else {
        planning_from_prev_node = true;
        std::vector<geometry_msgs::Pose> new_safety_path = res.path;
        for (int s = 0; s < req_pt_index; s++) {
          geometry_msgs::Pose p;
          p.position.x = prev_path[s]->position(0);
          p.position.y = prev_path[s]->position(1);
          p.position.z = prev_path[s]->position(2);
          new_safety_path.push_back(p);
        }
        mbtree_->setSafetyPath(new_safety_path);
        if (prev_path.size() > req_pt_index + 1) {
          for (int s = req_pt_index; s < prev_path.size(); s++) {
            new_prev_path.push_back(prev_path[s]);
          }
        } else {
          new_prev_path.push_back(prev_path[req_pt_index]);
          StateNode* curr_state_pt = new StateNode;
          curr_state_pt->position = current_state_.position;
          curr_state_pt->velocities = current_state_.velocities;
          new_prev_path.push_back(curr_state_pt);
        }
        return true;
      }
    } else
      return false;
  }
  return true;
  // else {ROS_WARN("Planner returned empty tree."); return false;}
}

void MBPlanner::odometryCallback(const nav_msgs::Odometry& odo) {
  this->current_pose = odo.pose.pose;
  StateNode state;
  state.position(0) = odo.pose.pose.position.x;
  state.position(1) = odo.pose.pose.position.y;
  state.position(2) = odo.pose.pose.position.z;

  state.velocities(0) = odo.twist.twist.linear.x;
  state.velocities(1) = odo.twist.twist.linear.y;
  state.velocities(2) = odo.twist.twist.linear.z;

  tf::Quaternion q;
  q[0] = odo.pose.pose.orientation.x;
  q[1] = odo.pose.pose.orientation.y;
  q[2] = odo.pose.pose.orientation.z;
  q[3] = odo.pose.pose.orientation.w;
  tf::Matrix3x3 mat(q);
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  state.yaw = yaw;
  state.pitch = pitch;
  state.roll = roll;

  current_state_ = state;

  mbtree_->set_state(state);
}

}  // explorer