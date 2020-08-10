/* This file has the definitions of the functions to build the tree,
find the exploration gain for all the paths and get the best path */

#include "mbplanner/mbtree.h"

namespace explorer {
namespace mbplanner {

MBTree::MBTree(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
#ifdef USE_OCTOMAP
  map_manager_ = new explorer::MapManagerOctomap(nh, nh_private);
#else
  map_manager_ =
      new explorer::MapManagerVoxblox<explorer::MapManagerVoxbloxServer,
                                      explorer::MapManagerVoxbloxVoxel>(nh, nh_private);
#endif

  // kd trees for the motion primitive tree and gain calculation
  this->kd_tree_ = kd_create(3);    // Gain calculation
  this->main_tree_ = kd_create(3);  // Motion primitive tree

  // Load Parameters
  ns = ros::this_node::getName();
  mb_params_.loadParams(ns + "/MBParams");
  robot_params_.loadParams(ns + "/RobotParams");
  sensor_params_.loadParams(ns + "/SensorParams");
  free_frustum_params_.loadParams(ns + "/FreeFrustumParams");
  local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local");
  global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global");

  // Object to store planner output charateristics (computation times, path
  // lenght etc.)
  stat_.reset(new SampleStatistic());

  next_root_ = NULL;
  // Initialize sampler distributions
  std::random_device rd;
  gen_.seed(rd());
  distribution_x_.reset(
      new std::uniform_real_distribution<>(mb_params_.ax_min, mb_params_.ax_max));
  distribution_y_.reset(
      new std::uniform_real_distribution<>(mb_params_.ay_min, mb_params_.ay_max));
  distribution_z_.reset(
      new std::uniform_real_distribution<>(mb_params_.az_min, mb_params_.az_max));

  // Visualization:
  visualizer_ = new Visualization(nh_, nh_private_);

  planner_logger_pub_ = nh_.advertise<planner_msgs::PlannerLogger>("mbplanner/logger", 10);
  free_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/freespace_pointcloud", 10);
  tree_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("local_tree", 10);

  // Other parameter initialization
  rostime_start_ = ros::Time::now();
  odometry_ready = false;
  first_odom = true;
  world_frame_id = "world";
  tree_root_ = new StateNode();
  tree_root_->position = current_state_.position;
  tree_root_->printit();
  StateVec root;
  root << tree_root_->position(0), tree_root_->position(1), tree_root_->position(2),
      tree_root_->yaw;
  visualizer_->visualizeWorkspace(root, global_space_params_, local_space_params_);

  // Timers
  periodic_timer_ = nh_.createTimer(ros::Duration(kTimerPeriod), &MBTree::timerCallback, this);
  // periodic_timer_free_pcl_ =
  //     nh_.createTimer(ros::Duration(0.1), &MBTree::freePointCloudtimerCallback, this);
}

MBTree::~MBTree() {}

void MBTree::deleteTree(StateNode* root) {
  if (root == NULL) return;

  std::queue<StateNode*> q;  // Create a queue
  q.push(root);
  while (!q.empty()) {
    int n = q.size();
    // If this node has children
    while (n > 0) {
      for (int i = 0; i < q.front()->children.size(); i++) q.push(q.front()->children[i]);
      delete q.front();
      q.pop();
      n--;
    }
  }
}

void MBTree::reset() {
  ROS_INFO("Reseting the tree.");

  this->viz_state_tree_.clear();
  stat_.reset(new SampleStatistic());
}

void MBTree::setRoot(StateNode root) { root_storage = root; }

void MBTree::setPrevPath(std::vector<StateNode*> p_path) { this->prev_path_ = p_path; }

void MBTree::resetExpDir() { exploring_direction_ = mb_params_.exp_dir; }

void MBTree::setSafetyPath(std::vector<StateNode*> new_path) { safe_path_ = new_path; }

void MBTree::setSafetyPath(std::vector<geometry_msgs::Pose> new_path) {
  std::vector<StateNode*> new_safety_path;
  for (int i = 0; i < new_path.size(); i++) {
    StateNode* state = new StateNode;
    state->position(0) = new_path[i].position.x;
    state->position(1) = new_path[i].position.y;
    state->position(2) = new_path[i].position.z;
    new_safety_path.push_back(state);
  }
  safe_path_ = new_safety_path;
}

int MBTree::buildTree(bool prev_point_plan, bool use_current_state, bool use_given_vel) {
  /*
  Args:
    prev_point_plan: Tree root is end point of previous path
    use_current_state: Tree root is current state. Used for planning after robot was stopped
  (start of mission, global planner triggered)
    use_given_vel: Use the velocity set by the setGivenVel() function
  Return:
    Number of vertices in the tree
  */
  ros::Time tim;
  START_TIMER(tim);
  // Invalid sample count
  int bad_samples = 0;
  int dist_bad_samples = 0;
  int bad_samples_full = 0;

  // Number of samples to be taken in that iteration
  int current_sample_steps = mb_params_.sample_steps;
  // Number of good samples in that iteration
  int good_samples = 0;
  int total_bad_samples = 0;

  // Set root node
  if (mb_params_.use_current_state > 0 || use_current_state) {
    StateNode* new_root = new StateNode;
    new_root->position = current_state_.position;
    new_root->parent = NULL;
    if (!use_given_vel)
      new_root->velocities = Eigen::Vector3d(0.0, 0.0, 0.0);
    else
      new_root->velocities = given_vel_;
    tree_root_ = new_root;
  } else {
    if (next_root_ == NULL) {  // Planning for first time
      StateNode* new_root = new StateNode;
      new_root->position = current_state_.position;
      new_root->parent = NULL;
      if (!use_given_vel)
        new_root->velocities = Eigen::Vector3d(0.0, 0.0, 0.0);
      else
        new_root->velocities = given_vel_;
      tree_root_ = new_root;
    } else {
      StateNode* new_root = new StateNode;
      new_root->position = root_storage.position;
      if (!use_given_vel)
        new_root->velocities = root_storage.velocities;
      else
        new_root->velocities = given_vel_;
      new_root->parent = NULL;
      if (prev_point_plan) {
        new_root->position = root_storage.position;
        if (!use_given_vel)
          new_root->velocities = Eigen::Vector3d(0.0, 0.0, 0.0);
        else
          new_root->velocities = given_vel_;
      }
      tree_root_ = new_root;
    }
  }
  tree_root_->dist_to_vertex = 0.0;
  tree_root_->yaw = current_state_.yaw;
  ROS_INFO("Tree root:");
  tree_root_->printit();
  final_path_.resize(0);

  // KdTree
  kd_free(main_tree_);
  main_tree_ = kd_create(3);
  kd_insert3(main_tree_, tree_root_->position(0), tree_root_->position(1),
             tree_root_->position(2), tree_root_);

  // For visualization:
  // Sensor FOV:
  StateVec root;
  root[0] = tree_root_->position(0);
  root[1] = tree_root_->position(1);
  root[2] = tree_root_->position(2);
  root[3] = tree_root_->yaw;
  visualizer_->visualizeSensorFOV(root, sensor_params_);
  // Planning workspace:
  visualizer_->visualizeWorkspace(root, global_space_params_, local_space_params_);

  // Variables for generating tree
  int iteration_count = 0;  // Used to determine how many control inputs to sample. It is equal
                            // to iter unless no valid samples are sampled in an iteration
  int iter = 0;             // Iteration of loop
  int tree_size = 0;
  // For sampling certain percentage of accelerations around the current exploration direction
  int exp_dir_samples =
      mb_params_.sample_steps / 4.0;  // 1/4 samples around current exploration
                                      // direction(for the 1t iteration)
  int exp_dir_samples_fixed = exp_dir_samples;
  double exp_delta = -0.1747;  // 10 degrees
  double exp_d_theta = -2 * (double)exp_delta / (double)(exp_dir_samples_fixed - 1);

  // Clear vol around root vertex if required
  explorer::MapManager::VoxelStatus voxel_state =
      map_manager_->getBoxStatus(tree_root_->position, robot_params_.size, true);
  if (voxel_state == explorer::MapManager::VoxelStatus::kOccupied) {
    ROS_WARN("ROOT INSIDE OBSTACLE");
    map_manager_->augmentFreeBox(tree_root_->position, robot_params_.size);
  }

  StateNode* prev_d_state = new StateNode;

  std::queue<StateNode*> q;
  q.push(tree_root_);

  while (iter < mb_params_.iterations) {
    int robot_box_free_id = 0;
    int robot_box_occupied_id = 0;
    int robot_box_unknown_id = 0;
    int n = q.size();
    if (good_samples == 0) q.push(tree_root_);
    int z_bad_samples = 0;
    int col_bad_samples = 0;
    int unknown_bad_samples = 0;
    bad_samples_full = 0;
    good_samples = 0;
    dist_bad_samples = 0;
    int dist_bad_samples_display = 0;
    while (n > 0) {
      bad_samples = 0;

      bool first_sample = true;

      // Set number of control inputs to be sampled for this node
      if (iteration_count == 0)
        current_sample_steps = mb_params_.sample_steps;
      else if (iteration_count == 1)
        current_sample_steps = mb_params_.later_iter_samples + mb_params_.second_lvl_samples;
      else
        current_sample_steps = mb_params_.later_iter_samples;
      double temp_sample_steps = current_sample_steps;

      prev_d_state = q.front();
      // Set control space
      if (mb_params_.use_angles == 0) {
        if (iteration_count >= 1) {
          distribution_x_.reset(new std::uniform_real_distribution<>(
              std::max(q.front()->acceleration(0) - mb_params_.a_cone, mb_params_.ax_min),
              std::min(q.front()->acceleration(0) + mb_params_.a_cone, mb_params_.ax_max)));
          distribution_y_.reset(new std::uniform_real_distribution<>(
              std::max(q.front()->acceleration(1) - mb_params_.a_cone, mb_params_.ay_min),
              std::min(q.front()->acceleration(1) + mb_params_.a_cone, mb_params_.ay_max)));
          distribution_z_.reset(new std::uniform_real_distribution<>(
              std::max(q.front()->acceleration(2) - mb_params_.a_cone, mb_params_.az_min),
              std::min(q.front()->acceleration(2) + mb_params_.a_cone, mb_params_.az_max)));
        } else {
          distribution_x_.reset(
              new std::uniform_real_distribution<>(mb_params_.ax_min, mb_params_.ax_max));
          distribution_y_.reset(
              new std::uniform_real_distribution<>(mb_params_.ay_min, mb_params_.ay_max));
          distribution_z_.reset(
              new std::uniform_real_distribution<>(mb_params_.az_min, mb_params_.az_max));
        }
      } else {
        double acc_dir = atan2(q.front()->acceleration(1), q.front()->acceleration(0));
        if (iteration_count >= 1) {
          distribution_x_.reset(
              new std::uniform_real_distribution<>(acc_dir - mb_params_.a_cone,
                                                   acc_dir + mb_params_.a_cone));  // Angle
          distribution_y_.reset(new std::uniform_real_distribution<>(
              mb_params_.ay_min, mb_params_.ay_max));  // Mag
          distribution_z_.reset(new std::uniform_real_distribution<>(
              std::max(q.front()->acceleration(2) - 0.8, mb_params_.az_min),
              std::min(q.front()->acceleration(2) + 0.8, mb_params_.az_max)));
        } else {
          distribution_x_.reset(new std::uniform_real_distribution<>(
              mb_params_.ax_min, mb_params_.ax_max));  // Angle
          distribution_y_.reset(new std::uniform_real_distribution<>(
              mb_params_.ay_min, mb_params_.ay_max));  // Mag
          distribution_z_.reset(
              new std::uniform_real_distribution<>(mb_params_.az_min, mb_params_.az_max));
        }
      }

      // Sample "current_sample_steps" number of samples
      for (int c = 0; c < current_sample_steps; c++) {
        double ax, ay, az;
        if (iteration_count >= 1) {
          if (iteration_count <= mb_params_.branch_limit) {
            if (first_sample) {
              first_sample = true;
              ax = q.front()->acceleration(0);
              ay = q.front()->acceleration(1);
              az = q.front()->acceleration(2);
            } else {
              if (mb_params_.use_angles == 0) {
                ax = (*distribution_x_)(gen_);
                ay = (*distribution_y_)(gen_);
                az = (*distribution_z_)(gen_);
              } else {
                double theta = (*distribution_x_)(gen_);
                double mag = (*distribution_y_)(gen_);
                ax = mag * cos(theta);
                ay = mag * sin(theta);
                az = (*distribution_z_)(gen_);
              }
            }
          } else {
            if (mb_params_.use_angles == 0) {
              ax = (*distribution_x_)(gen_);
              ay = (*distribution_y_)(gen_);
              az = (*distribution_z_)(gen_);
            } else {
              double theta = (*distribution_x_)(gen_);
              double mag = (*distribution_y_)(gen_);
              // truncate_yaw(theta);
              ax = mag * cos(theta);
              ay = mag * sin(theta);
              az = (*distribution_z_)(gen_);
            }
            current_sample_steps = 1;
          }
        } else {
          if (exp_dir_samples > 0) {
            double theta = exploring_direction_ + exp_delta + exp_d_theta;
            if (mb_params_.use_angles == 0) {
              ax = mb_params_.ax_max * cos(exploring_direction_ + exp_delta);
              ay = mb_params_.ay_max * sin(exploring_direction_ + exp_delta);
            } else {
              ax = mb_params_.ay_max * cos(exploring_direction_ + exp_delta);
              ay = mb_params_.ay_max * sin(exploring_direction_ + exp_delta);
            }
            az = (*distribution_z_)(gen_);
            exp_delta = exp_delta + exp_d_theta;
            exp_dir_samples--;
          } else {
            if (c == mb_params_.sample_steps - 1 && mb_params_.x_bias > 0) {
              if (mb_params_.use_angles == 0)
                ax = mb_params_.ax_max;
              else
                ax = mb_params_.ay_max;
              ay = 0.0;
              az = (*distribution_z_)(gen_);
            } else {
              if (mb_params_.use_angles == 0) {
                ax = (*distribution_x_)(gen_);
                ay = (*distribution_y_)(gen_);
              } else {
                double theta = (*distribution_x_)(gen_);
                double mag = (*distribution_y_)(gen_);
                ax = mag * cos(theta);
                ay = mag * sin(theta);
              }
              az = (*distribution_z_)(gen_);
            }
          }
        }
        Eigen::Vector3d a(ax, ay, az);  // Final sampled acceleration

        double t_c;  // Just in case if this needs to be variable in the future
        if (mb_params_.var_no_of_sampling > 0) {
          if (iteration_count == 0)
            t_c = mb_params_.t / 2.0;
          else
            t_c = mb_params_.t;
        } else
          t_c = mb_params_.t;

        prev_d_state = q.front();

        bool hit = false;
        double curve_dist = 0.0;

        std::vector<StateNode*> temp_inter_state_tree_;  // This stores the intermediate states
                                                         // of the motion primitive

        // Calculate the motion primitive for the sampled acceleration
        // Total time duration of the primitive is set by the param t(from the config file)
        // The primitive is divided into steps of time duration dt (set from the config file)
        // From the previous state, using the sampled control input,
        // the next state after time dt is calculated,
        // checked if it lies in the free space,
        // if yes, then this is used to calculate the next step and the process repeats till
        // time t.
        // if no, then this sample is discarded and the loop count is decremented to resample
        // an acceleration
        // This finishes the primitive
        for (int k = 0; k < (int)(t_c / mb_params_.dt); k++) {
          StateNode* d_state = new StateNode;
          d_state->velocities = prev_d_state->velocities + a * mb_params_.dt;
          if (d_state->velocities.norm() > mb_params_.v_max)
            d_state->velocities =
                (mb_params_.v_max / d_state->velocities.norm()) * d_state->velocities;

          d_state->position = prev_d_state->position +
                              prev_d_state->velocities * mb_params_.dt +
                              0.5 * mb_params_.dt * mb_params_.dt * a;
          // Is the state within the local and global bounding box:
          if (!inside_global_box(d_state->position)) {
            hit = true;
            break;
          }
          if (!inside_local_box(d_state->position)) {
            hit = true;
            break;
          }

          // Collision check:
          explorer::MapManager::VoxelStatus voxel_state =
              map_manager_->getBoxStatus(d_state->position, robot_params_.size, false);
          if (voxel_state == explorer::MapManager::VoxelStatus::kOccupied)  // Collision
          {
            col_bad_samples++;
            if (k < mb_params_.min_edge_length) hit = true;
            break;
          } else if (voxel_state == explorer::MapManager::VoxelStatus::kUnknown) {
            unknown_bad_samples++;
            if (iter < mb_params_.iterations - 1)
              if (k < mb_params_.min_edge_length) hit = true;
            break;
          }  // Went into unexplored environment
          // In free space
          curve_dist += (prev_d_state->position - d_state->position).norm();

          d_state->dist_to_vertex = prev_d_state->dist_to_vertex +
                                    (prev_d_state->position - d_state->position).norm();
          d_state->acceleration = a;
          if (k < (int)(t_c / mb_params_.dt) - 1) temp_inter_state_tree_.push_back(d_state);
          prev_d_state = (d_state);
        }

        // Had collision:
        if (hit) {
          first_sample = false;
          temp_inter_state_tree_.clear();
          c--;
          bad_samples++;
          total_bad_samples++;
          if (iteration_count != 0) {
            if (bad_samples >= mb_params_.max_sampling_iters) {
              bad_samples_full++;
              bad_samples = 0;
              c++;
            }
          } else {
            if (bad_samples >= mb_params_.max_sampling_iters * 4) {
              bad_samples_full++;
              bad_samples = 0;
              c++;
            }
          }
          hit = false;
          continue;
        }

        StateNode* temp_new_state = prev_d_state;
        kdres* nearest = kd_nearest3(main_tree_, temp_new_state->position(0),
                                     temp_new_state->position(1), temp_new_state->position(2));
        StateNode* nn = (StateNode*)kd_res_item_data(nearest);
        if (iteration_count > 1) {
          double min_dist;
          if (iteration_count < 2)
            min_dist = mb_params_.min_vertex_separation / 2.0;
          else
            min_dist = mb_params_.min_vertex_separation;
          if ((nn->position - temp_new_state->position).norm() < min_dist) {
            first_sample = false;
            temp_inter_state_tree_.clear();
            c--;
            dist_bad_samples++;
            total_bad_samples++;
            dist_bad_samples_display++;
            if (iteration_count != 0) {
              if (dist_bad_samples >= mb_params_.max_sampling_iters) {
                bad_samples_full++;
                dist_bad_samples = 0;
                c++;
              }
            } else {
              if (dist_bad_samples >= mb_params_.max_sampling_iters * 2) {
                bad_samples_full++;
                dist_bad_samples = 0;
                c++;
              }
            }
            kd_res_free(nearest);
            continue;
          }
        }

        StateNode* new_state = new StateNode;
        new_state->position = prev_d_state->position;
        new_state->velocities = prev_d_state->velocities;
        // new_state->parent->printit();
        new_state->parent = q.front();
        for (int k = 0; k < temp_inter_state_tree_.size(); k++)
          new_state->minions.push_back(temp_inter_state_tree_[k]);
        temp_inter_state_tree_.clear();
        new_state->dist_to_vertex = q.front()->dist_to_vertex + curve_dist;
        new_state->acceleration = a;
        new_state->special_point = true;
        new_state->yaw = atan2((new_state->position(1) - new_state->parent->position(1)),
                               (new_state->position(0) - new_state->parent->position(0)));
        q.front()->children.push_back(new_state);

        kd_insert3(main_tree_, new_state->position(0), new_state->position(1),
                   new_state->position(2), new_state);

        good_samples++;
        tree_size++;
      }  // Current sample loop
      for (int i = 0; i < q.front()->children.size(); i++) q.push(q.front()->children[i]);
      n--;
      q.pop();
    }  // while(n>0)
    if (good_samples > 0) iteration_count++;
    iter++;
  }  // while(iteration_count < mb_params_.iterations)

  stat_->build_tree_time = GET_ELAPSED_TIME(tim);

  // Visualize tree
  visualizer_->visualizeMBTree(tree_root_);
  return tree_size;
}

void MBTree::set_state(StateNode currentState) {
  odometry_ready = true;
  this->current_state_ = currentState;
}

void MBTree::setGivenVel(Eigen::Vector3d vel) {
  if (vel.norm() <= mb_params_.v_max)
    given_vel_ = vel;
  else
    given_vel_ = (mb_params_.v_max / vel.norm()) * vel;
}

std::vector<Eigen::Vector3d> MBTree::getEigenPath(std::vector<StateNode*> statenode_path) {
  std::vector<Eigen::Vector3d> ret_path;
  for (int i = 0; i < statenode_path.size(); i++) {
    ret_path.push_back(statenode_path[i]->position);
  }
  return ret_path;
}

std::vector<geometry_msgs::Pose> MBTree::getPosePath(std::vector<StateNode*> path) {
  std::vector<geometry_msgs::Pose> ret_path;
  for (auto p : path) {
    geometry_msgs::Pose ret_p;
    ret_p.position.x = p->position(0);
    ret_p.position.y = p->position(1);
    ret_p.position.z = p->position(2);
    ret_p.orientation.x = 0.0;
    ret_p.orientation.y = 0.0;
    ret_p.orientation.z = 0.0;
    ret_p.orientation.w = 1.0;
    ret_path.push_back(ret_p);
  }
  return ret_path;
}

// Exploration gain calculations
bool MBTree::evaluateGraph() {
  int FIXED_VERTS = 5;
  bool gstatus = true;

  visualization_msgs::MarkerArray safe_paths_marker;
  visualization_msgs::Marker safe_paths;
  safe_paths.header.stamp = ros::Time::now();
  safe_paths.header.seq = 0;
  safe_paths.header.frame_id = world_frame_id;
  safe_paths.id = 0;
  safe_paths.ns = "edges";
  safe_paths.action = visualization_msgs::Marker::ADD;
  safe_paths.type = visualization_msgs::Marker::LINE_LIST;
  safe_paths.scale.x = 0.08;
  safe_paths.color.r = 200.0 / 255.0;
  safe_paths.color.g = 00.0 / 255.0;
  safe_paths.color.b = 0.0;
  safe_paths.color.a = 1.0;
  safe_paths.lifetime = ros::Duration(0.0);
  safe_paths.frame_locked = false;

  // FInding leaves and calculating volumetric gain
  leaf_vertices_ = findLeafVertices();
  std::vector<StateNode*> safe_leaves;
  ROS_INFO("Number of leaves: %d", leaf_vertices_.size());

  // Calculating full path gains:
  START_TIMER(ttime);
  if (leaf_vertices_.size() == 0) {
    ROS_WARN("No leaf found");
    gstatus = false;
    return gstatus;
  }

  // Gain evaluation for valid paths, starting from the leaf to the root.

  double best_gain = 0;
  StateNode* best_path_leaf;
  int num_leaf_vertices_ = leaf_vertices_.size();
  double len_min = 1.0;
  std::vector<std::pair<StateNode*, float>> paths;
  std::vector<StateNode*> backward_leaves;
  for (int i = 0; i < num_leaf_vertices_; ++i) {
    bool good = false;

    std::vector<StateNode*> path;
    path = getShortestPath(leaf_vertices_[i], false);

    int path_size = path.size();
    if (path_size > 1) {
      // At least 2 vertices: root + leaf.
      double path_gain = 0;
      double lambda = mb_params_.path_length_penalty;
      double path_length = 0;

      /**** Path Length ****/
      std::reverse(path.begin(), path.end());
      for (int ind = 1; ind < path.size(); ind++) {
        path_length += (path[ind]->position - path[ind - 1]->position).norm();
        path_gain += path[ind]->vol_gain.gain * exp(-lambda * path_length);

        double min_vert_dist = 9999.0;
        for (int j = 1; j < ind; j++) {
          double curr_vert_dist = (path[ind]->position - path[j]->position).norm();
          if (curr_vert_dist < min_vert_dist) min_vert_dist = curr_vert_dist;
        }
      }

      /* Min Path Length Check (min 1.0m) */
      if (path_length <= mb_params_.min_path_length) continue;

      /**** Safety ****/
      float min_obst_distance = 999.99;
      float mean_obst_distance = 0;
#ifndef USE_OCTOMAP
      std::vector<StateNode*> f_path = getShortestPath(leaf_vertices_[i], false);
      for (int ind = 0; ind < f_path.size() - 2; ind++) {
        float current_obst_dist;
        current_obst_dist = map_manager_->getVoxelDistance(Eigen::Vector3d(
            f_path[ind]->position(0), f_path[ind]->position(1), f_path[ind]->position(2)));
        if (current_obst_dist < min_obst_distance) min_obst_distance = current_obst_dist;
        mean_obst_distance += current_obst_dist;
      }
      mean_obst_distance = mean_obst_distance / (f_path.size() - 2);
#endif
      leaf_vertices_[i]->min_obst_dist = min_obst_distance;
      leaf_vertices_[i]->mean_obstacle_dist = mean_obst_distance;

      /* Safety Path Calc and Viz */
      leaf_vertices_[i]->obst_steps = predict_hit((leaf_vertices_[i]));
      if (leaf_vertices_[i]->obst_steps < 0)
        path_gain = 0.0;
      else
        safe_leaves.push_back(leaf_vertices_[i]);

      /**** Path Direction ****/
      double lambda2 = mb_params_.path_direction_penalty;
      std::vector<Eigen::Vector3d> path_list;
      for (auto p : path) path_list.insert(path_list.begin(), p->position);
      double path_direction = explorer::Trajectory::estimateDirectionFromPath(path_list);
      double dyaw = path_direction - exploring_direction_;
      if (dyaw > M_PI)
        dyaw -= 2 * M_PI;
      else if (dyaw < -M_PI)
        dyaw += 2 * M_PI;

      double kDiffAngleForwardThres = mb_params_.forward_angle * M_PI / 180.0;
      if (std::abs(dyaw) > kDiffAngleForwardThres) {
        leaf_vertices_[i]->final_gain = path_gain;
        backward_leaves.push_back(leaf_vertices_[i]);
        // continue;
      }
      path_gain *= exp(-lambda2 * std::abs(dyaw));

      leaf_vertices_[i]->final_gain = path_gain;

      if (path_gain > best_gain) {
        best_gain = path_gain;
        best_path_leaf = leaf_vertices_[i];
      }

    }  // if(path_size > 1)

  }  // loop over all leaves
  if (num_leaf_vertices_ == 1) {
    best_path_leaf = leaf_vertices_[0];
    best_gain = leaf_vertices_[0]->vol_gain.gain;
  }

  if (best_gain > 0) {
    best_vertex_ = best_path_leaf;
    gstatus = true;
  } else  // Zero gain for all leaves
  {
    for (int i = 0; i < backward_leaves.size(); i++) {
      if (backward_leaves[i]->final_gain > best_gain) {
        best_gain = backward_leaves[i]->final_gain;
        // best_path_id = id;
        best_path_leaf = backward_leaves[i];
      }
    }
    if (best_gain > 0) {
      best_vertex_ = best_path_leaf;
      gstatus = true;
    } else {
      gstatus = false;
      ROS_WARN("No positive gain for any vertex");
      return gstatus;
    }
  }

  // Serialise tree and publish for adding to global graph
  std::vector<StateNode*> clustered_leaves;
  std::vector<SerializeVertex> vertices;
  clusterPaths(clustered_leaves, 0.5);
  getGlobalGraphPaths(vertices, clustered_leaves);
  serializeAndPublish(vertices);

  // safe_paths_marker.markers.push_back(safe_paths);
  // safety_path_viz_pub.publish(safe_paths_marker);

  std::vector<Eigen::Vector3d> curr_best_path =
      getEigenPath(getShortestPath(best_vertex_, true));
  orig_path_ = getPosePath(getShortestPath(best_vertex_, true));

  std::vector<std::vector<geometry_msgs::Pose>> valid_similar_paths, invalid_similar_paths;
  actual_best_path = getPosePath(getShortestPath(best_vertex_, true));
  int similar_path_count = 0;

  /* Finding Similar paths and selecting safest */
  for (int i = 0; i < safe_leaves.size(); i++) {
    bool similar = explorer::Trajectory::compareTwoTrajectories(
        curr_best_path, getEigenPath(getShortestPath(safe_leaves[i], true)),
        mb_params_.similarity_dist);
    if (similar) {
      similar_path_count++;
      valid_similar_paths.push_back(getPosePath(getShortestPath(safe_leaves[i], false)));
      if (safe_leaves[i]->mean_obstacle_dist > best_path_leaf->mean_obstacle_dist) {
        best_path_leaf = safe_leaves[i];
      }
    }
  }

  // Visualize similar paths
  visualizer_->visualizeSimilarPaths(valid_similar_paths, invalid_similar_paths);
  // Visualize safety paths and exploration gains
  visualizer_->visualizeMBTreeEvaluation(leaf_vertices_);
  best_vertex_ = best_path_leaf;
  stat_->evaluate_graph_time = GET_ELAPSED_TIME(ttime);
  // stat_->total_time += stat_->evaluate_graph_time;

  safe_path_ = best_vertex_->safe_path;

  int text_id = 0;

  return gstatus;
}

void MBTree::clusterPaths(std::vector<StateNode*>& clustered_leaves, double dist_threshold) {
  std::vector<std::vector<Eigen::Vector3d>> cluster_paths;
  for (int i = 0; i < leaf_vertices_.size(); i++) {
    if (leaf_vertices_[i]->type == VertexType::kFrontier) {
      std::vector<StateNode*> curr_path = getShortestPath(leaf_vertices_[i], true);
      std::vector<Eigen::Vector3d> curr_path_eigen = getEigenPath(curr_path);
      bool found_neighbor = false;
      for (int j = 0; j < cluster_paths.size(); j++) {
        if (Trajectory::compareTwoTrajectories(curr_path_eigen, cluster_paths[j],
                                               dist_threshold)) {
          found_neighbor = true;
          break;
        }
      }
      if (!found_neighbor) {
        cluster_paths.push_back(curr_path_eigen);
        clustered_leaves.push_back(leaf_vertices_[i]);
      }
    }
  }
}

void MBTree::getGlobalGraphPaths(std::vector<SerializeVertex>& vertices,
                                 std::vector<StateNode*> clustered_leaves) {
  int id_counter = 0;

  for (int i = 0; i < clustered_leaves.size(); i++) {
    std::vector<StateNode*> path;
    path = getShortestPath(clustered_leaves[i], false);
    for (int j = 0; j < path.size(); j++) {
      SerializeVertex new_vert;
      new_vert = getSerVert(path[j]);

      bool exists = false;
      int original_id = 0;
      new_vert.id = id_counter++;
      if (j != path.size() - 1) new_vert.parent_id = new_vert.id + 1;
      vertices.push_back(new_vert);
    }
  }
}

SerializeVertex MBTree::getSerVert(StateNode* vert_ip) {
  SerializeVertex vert;
  vert.id = 0;
  vert.state[0] = vert_ip->position(0);
  vert.state[1] = vert_ip->position(1);
  vert.state[2] = vert_ip->position(2);
  vert.state[3] = vert_ip->yaw;
  vert.parent_id = -1;
  vert.vol_gain.gain = vert_ip->vol_gain.gain;
  vert.vol_gain.num_unknown_voxels = vert_ip->vol_gain.num_unknown_voxels;
  vert.vol_gain.num_free_voxels = vert_ip->vol_gain.num_free_voxels;
  vert.vol_gain.num_occupied_voxels = vert_ip->vol_gain.num_occupied_voxels;
  vert.vol_gain.accumulative_gain = vert_ip->vol_gain.accumulative_gain;
  vert.vol_gain.is_frontier = vert_ip->vol_gain.is_frontier;
  if (vert_ip->type == VertexType::kFrontier)
    vert.type = 2;
  else if (vert_ip->type == VertexType::kVisited)
    vert.type = 1;
  else
    vert.type = 0;

  return vert;
}

void MBTree::serializeAndPublish(std::vector<SerializeVertex> vertices) {
  namespace ser = ros::serialization;

  std::vector<SerializeVertex> vertices_pub = vertices;
  uint32_t serial_size = ros::serialization::serializationLength(vertices_pub);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ser::OStream stream(buffer.get(), serial_size);
  ser::serialize(stream, vertices_pub);

  std_msgs::UInt8MultiArray pub_array;
  for (int i = 0; i < serial_size; i++) pub_array.data.push_back(buffer[i]);

  tree_pub_.publish(pub_array);
}

bool MBTree::is_line_good(StateNode* start, StateNode* end) {
  // start->printit();
  // end->printit();
  double theta, phi, R;
  theta = atan2(end->position(1) - start->position(1), end->position(0) - start->position(0));
  double Dx = end->position(0) - start->position(0);
  double Dy = end->position(1) - start->position(1);
  R = sqrt(Dx * Dx + Dy * Dy);

  phi = atan2(end->position(2) - start->position(2), R);

  double voxel_size = 0.6;
  int iters = (int)((end->position - start->position).norm() / voxel_size);

  for (int i = 0; i < iters; i++) {
    Eigen::Vector3d new_point;
    new_point(0) = start->position(0) + i * voxel_size * cos(phi) * cos(theta);
    new_point(1) = start->position(1) + i * voxel_size * cos(phi) * sin(theta);
    new_point(2) = start->position(2) + i * voxel_size * sin(phi);
    if (explorer::MapManager::VoxelStatus::kOccupied ==
        map_manager_->getBoxStatus(new_point, robot_params_.size, true))
      return false;
  }
  return true;
}

bool MBTree::inside_global_box(Eigen::Vector3d position) {
  if (position(0) >= global_space_params_.min_val[0] + robot_params_.size[0] / 2.0 &&
      position(0) <= global_space_params_.max_val[0] - robot_params_.size[0] / 2.0) {
    if (position(1) >= global_space_params_.min_val[1] + robot_params_.size[1] / 2.0 &&
        position(1) <= global_space_params_.max_val[1] - robot_params_.size[1] / 2.0) {
      if (position(2) >= global_space_params_.min_val[2] + robot_params_.size[2] / 2.0 &&
          position(2) <= global_space_params_.max_val[2] - robot_params_.size[2] / 2.0) {
        return true;
      } else
        return false;
    } else
      return false;
  } else
    return false;
}

bool MBTree::inside_local_box(Eigen::Vector3d position) {
  if (position(0) - tree_root_->position(0) >= local_space_params_.min_val[0] &&
      position(0) - tree_root_->position(0) <= local_space_params_.max_val[0]) {
    if (position(1) - tree_root_->position(1) >= local_space_params_.min_val[1] &&
        position(1) - tree_root_->position(1) <= local_space_params_.max_val[1]) {
      if (position(2) - tree_root_->position(2) >= local_space_params_.min_val[2] &&
          position(2) - tree_root_->position(2) <= local_space_params_.max_val[2]) {
        return true;
      } else
        return false;
    } else
      return false;
  } else
    return false;
}

void MBTree::computeDistanceMap(Eigen::Vector3d& map_center, Eigen::Vector3d& map_size) {
  // Center detection
  ros::Time time_temp;
  START_TIMER(time_temp);

  std::vector<Eigen::Vector3d> occupied_voxels;
  std::vector<Eigen::Vector3d> free_voxels;
  map_manager_->extractLocalMap(map_center, map_size, occupied_voxels, free_voxels);

  kdtree* kd_tree_dm;
  kd_tree_dm = kd_create(3);
  obs_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  for (auto& v : occupied_voxels) {
    kd_insert3(kd_tree_dm, v.x(), v.y(), v.z(), &v);
    obs_pcl_->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
  }

  // Update distance map for all vertices in the local graph.
  if (mb_params_.safety_aware_enable) {
    const double cutoff_dist = mb_params_.safety_distance;
    std::vector<StateNode*> all_nodes;
    std::queue<StateNode*> q;  // Create a queue
    q.push(tree_root_);
    while (!q.empty()) {
      int n = q.size();
      // If this node has children
      while (n > 0) {
        StateNode* p = q.front();
        for (int i = 0; i < p->children.size(); i++) q.push(p->children[i]);
        all_nodes.push_back(p);
        q.pop();
        n--;
      }
    }
    for (int i = 0; i < all_nodes.size(); ++i) {
      StateNode* sampled_vertex = (all_nodes[i]);
      kdres* nearest = kd_nearest3(kd_tree_dm, sampled_vertex->position.x(),
                                   sampled_vertex->position.y(), sampled_vertex->position.z());
      if (nearest) {
        Eigen::Vector3d* pt;
        pt = (Eigen::Vector3d*)kd_res_item_data(nearest);
        Eigen::Vector3d dist;
        dist << sampled_vertex->position.x() - pt->x(), sampled_vertex->position.y() - pt->y(),
            sampled_vertex->position.z() - pt->z();
        sampled_vertex->dm = dist.norm();
        if (sampled_vertex->dm > cutoff_dist) sampled_vertex->dm = cutoff_dist;
      }
      kd_res_free(nearest);
    }
  }

  // Compute the whole distance map in local space.
  // std::vector<std::pair<Eigen::Vector3d, double>> dm_free_voxels;
  // for (auto& v : free_voxels) {
  //   kdres* nearest = kd_nearest3(kd_tree_dm, v.x(), v.y(), v.z());
  //   Eigen::Vector3d* pt;
  //   pt = (Eigen::Vector3d*)kd_res_item_data(nearest);
  //   Eigen::Vector3d dist;
  //   dist << v.x() - pt->x(), v.y() - pt->y(), v.z() - pt->z();
  //   dm_free_voxels.push_back(std::make_pair(v, dist.norm()));
  //   kd_res_free(nearest);
  // }

  kd_free(kd_tree_dm);

  double dtime_temp = GET_ELAPSED_TIME(time_temp);
  ROS_WARN("Compute the distance map in %f(s)", dtime_temp);
  // visualization_->visualizePCL(obs_pcl_.get());
  // visualization_->visualizeCostMap(dm_free_voxels,
  // map_manager_->getResolution());
}

double MBTree::calculate_curvature(std::vector<StateNode*> path) {
  double max_curvature = 0.0;
  for (int i = 2; i < path.size(); i++) {
    double a, b, c, k, den, curvature;
    a = (path[i]->position - path[i - 1]->position).norm();
    b = (path[i]->position - path[i - 2]->position).norm();
    c = (path[i - 2]->position - path[i - 1]->position).norm();

    k = sqrt((a + (b + c)) * (c - (a - b)) * (c + (a - b)) *
             (a + (b - c)));  // 4*area_of_triangle(abc)
    den = a * b * c;
    if (den == 0.0)
      curvature = 0.0;
    else
      curvature = k / den;

    if (curvature > max_curvature) max_curvature = curvature;
  }

  return max_curvature;
}

int MBTree::predict_hit(StateNode* leaf) {
  StateNode* prev_d_state = leaf;
  bool hit = false;
  double a_tot_max;
  if (mb_params_.use_angles == 0) {
    Eigen::Vector3d a_max(mb_params_.ax_max, mb_params_.ay_max, mb_params_.az_max);
    a_tot_max = a_max.norm();
  } else
    a_tot_max = mb_params_.ay_max;

  Eigen::Vector3d a(0.0, 0.0, 0.0);

  int obst_dir_count = 0;
  double ax = -a_tot_max * (leaf->velocities(0) / leaf->velocities.norm());
  double ay = -a_tot_max * (leaf->velocities(1) / leaf->velocities.norm());
  double az = -a_tot_max * (leaf->velocities(2) / leaf->velocities.norm());

  Eigen::Vector3d a_test(ax, ay, az);
  double v_norm = mb_params_.v_max;
  std::vector<StateNode*> temp_path;
  while (v_norm > 0.1) {
    StateNode* d_state = new StateNode;
    double dt_temp = 0.05;
    d_state->velocities = prev_d_state->velocities + a_test * dt_temp;
    d_state->position = prev_d_state->position + prev_d_state->velocities * dt_temp +
                        0.5 * a_test * dt_temp * dt_temp;

    explorer::MapManager::VoxelStatus voxel_state =
        map_manager_->getBoxStatus(d_state->position, (robot_params_.size), false);
    if (voxel_state == explorer::MapManager::VoxelStatus::kOccupied)  // Collision
    {
      hit = true;
      break;
    }
    if (voxel_state == explorer::MapManager::VoxelStatus::kUnknown) {
      hit = true;
      break;
    }
    v_norm = d_state->velocities.norm();
    temp_path.push_back(d_state);
    prev_d_state = d_state;
  }

  prev_d_state = leaf;

  if (hit == true) {
    bool found = false;
    v_norm = mb_params_.v_max;
    double yaw = atan2(leaf->velocities(1), leaf->velocities(0));
    for (int i = 0; i < mb_params_.safe_path_samples; i++) {
      v_norm = mb_params_.v_max;
      double theta = yaw + M_PI / 2 + M_PI / 4 + i * M_PI / mb_params_.safe_path_samples;
      if (theta > 2 * M_PI) theta -= 2 * M_PI;

      double ax = a_tot_max * cos(theta);
      double ay = a_tot_max * sin(theta);
      double az = 0.0;

      Eigen::Vector3d a_test(ax, ay, az);
      StateNode* prev_d_state = leaf;
      bool hit2 = false;
      std::vector<StateNode*> temp_path2;
      int counter = 0;
      double dt_temp = 0.1;
      while (v_norm > 0.1 && counter < (int)(1.0 / dt_temp)) {
        StateNode* d_state = new StateNode;
        d_state->velocities = prev_d_state->velocities + a_test * dt_temp;
        d_state->position = prev_d_state->position + prev_d_state->velocities * dt_temp +
                            0.5 * a_test * dt_temp * dt_temp;

        explorer::MapManager::VoxelStatus voxel_state =
            map_manager_->getBoxStatus(d_state->position, (robot_params_.size), false);

        if (voxel_state == explorer::MapManager::VoxelStatus::kOccupied)  // Collision
        {
          hit2 = true;
          break;
        }
        if (voxel_state == explorer::MapManager::VoxelStatus::kUnknown) {
          hit2 = true;
          break;
        }
        v_norm = d_state->velocities.norm();
        temp_path2.push_back(d_state);
        counter++;
        prev_d_state = d_state;
      }
      if (hit2)
        continue;
      else {
        leaf->safe_path = temp_path2;
        found = true;
        break;
      }
    }
    if (!found) return -1;

    return 1;
  }
  leaf->safe_path = temp_path;
  temp_path.clear();
  return 1;  // Only reject infeasible trajectories
}

void MBTree::computeExplorationGain() {
  const int id_viz = 20;  // random vertex to show volumetric gain.
  ros::Time tim;
  START_TIMER(tim);
  // Making array of all nodes
  std::vector<StateNode*> all_nodes;
  std::queue<StateNode*> q;  // Create a queue
  q.push(tree_root_);
  while (!q.empty()) {
    int n = q.size();
    // If this node has children
    while (n > 0) {
      StateNode* p = q.front();
      for (int i = 0; i < p->children.size(); i++) q.push(p->children[i]);
      all_nodes.push_back(p);
      q.pop();
      n--;
    }
  }
  for (int i = 0; i < all_nodes.size(); i++) {
    StateVec v_state;
    v_state(0) = all_nodes[i]->position(0);
    v_state(1) = all_nodes[i]->position(1);
    v_state(2) = all_nodes[i]->position(2);
    v_state(3) = 0.0;
    computeVolumetricGainRayModel(v_state, all_nodes[i]->vol_gain, false);

    if (all_nodes[i]->vol_gain.is_frontier) all_nodes[i]->type = VertexType::kFrontier;
  }
  stat_->compute_exp_gain_time = GET_ELAPSED_TIME(tim);
}

VolumetricGain MBTree::computeIndividualExplorationGain(StateNode& leaf) {
  const int id_viz = 20;  // random vertex to show volumetric gain.

  StateVec v_state;
  VolumetricGain v_gain;
  v_gain = leaf.vol_gain;
  v_state(0) = leaf.position(0);
  v_state(1) = leaf.position(1);
  v_state(2) = leaf.position(2);
  v_state(3) = leaf.yaw;
  computeVolumetricGainRayModel(v_state, v_gain, false);
  return v_gain;
}

void MBTree::computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                           bool vis_en) {
  vgain.reset();

  // Scan winthin a local space and sensor range.
  // Compute the local bound.
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;
  // if (local_space_params_.type == BoundedSpaceType::kSphere) {
  //   for (int i = 0; i < 3; ++i) {
  //       bound_min[i] = current_state_.position - local_space_params_.radius -
  //                    local_space_params_.radius_extension;
  //       bound_max[i] = current_state_.position + local_space_params_.radius +
  //                    local_space_params_.radius_extension;
  //   }
  StateVec root_state_vec;
  root_state_vec(0) = tree_root_->position(0);
  root_state_vec(1) = tree_root_->position(1);
  root_state_vec(2) = tree_root_->position(2);
  root_state_vec(3) = 0.0;
  if (local_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; ++i) {
      bound_min[i] = root_state_vec[i] + local_space_params_.min_val[i] +
                     local_space_params_.min_extension[i];
      bound_max[i] = root_state_vec[i] + local_space_params_.max_val[i] +
                     local_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Local space is not defined.");
    return;
  }

  // Refine the bound with global bound.
  // if (global_space_params_.type == BoundedSpaceType::kSphere) {
  //   for (int i = 0; i < 3; i++) {
  //     bound_min[i] =
  //         std::max(bound_min[i], -global_space_params_.radius -
  //                                    global_space_params_.radius_extension);
  //     bound_max[i] =
  //         std::min(bound_max[i], global_space_params_.radius +
  //                                    global_space_params_.radius_extension);
  //   }
  if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = std::max(bound_min[i], global_space_params_.min_val[i] +
                                                global_space_params_.min_extension[i]);
      bound_max[i] = std::min(bound_max[i], global_space_params_.max_val[i] +
                                                global_space_params_.max_extension[i]);
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, explorer::MapManager::VoxelStatus>> voxel_log;
  int raw_unk_voxels_count = 0;

  for (int ind = 0; ind < mb_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = mb_params_.exp_sensor_list[ind];
    // Refine the bound within an effective range.
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i], state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, explorer::MapManager::VoxelStatus>> voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;

    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state, multiray_endpoints);
    map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp, voxel_log_tmp);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Have to remove those not belong to the local bound.
    // At the same time check if this is frontier.
    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      explorer::MapManager::VoxelStatus vs = vl.second;
      if (vs == explorer::MapManager::VoxelStatus::kUnknown) ++raw_unk_voxels_count;
      int j = 0;
      for (j = 0; j < 3; j++) {
        if ((voxel[j] < bound_min[j]) || (voxel[j] > bound_max[j])) break;
      }
      if (j == 3) {
        // valid voxel.
        if (vs == explorer::MapManager::VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == explorer::MapManager::VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == explorer::MapManager::VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR("Unsupported voxel type.");
        }
        if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
      }
    }

    gain_log.push_back(
        std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels));
    // Check if it is a potential frontier.
    if (sensor_params_.sensor[sensor_name].isFrontier(num_unknown_voxels *
                                                      map_manager_->getResolution())) {
      vgain.is_frontier = true;  // Event E2
    }
  }
  // Return gain values.
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * mb_params_.unknown_voxel_gain +
                  num_free_voxels * mb_params_.free_voxel_gain +
                  num_occupied_voxels * mb_params_.occupied_voxel_gain;
  }
}

std::vector<StateNode*> MBTree::findLeafVertices() {
  ros::Time tim;
  START_TIMER(tim);

  std::vector<StateNode*> leaves;
  std::vector<StateNode *> all_nodes, final_all_nodes;
  std::queue<StateNode*> q;  // Create a queue
  q.push(tree_root_);
  while (!q.empty()) {
    int n = q.size();
    // If this node has children
    while (n > 0) {
      StateNode* p = q.front();
      all_nodes.push_back(p);
      if (q.front()->children.size() == 0) {
        p->vol_gain = computeIndividualExplorationGain((*p));  // No clustering
        if (p->vol_gain.is_frontier) p->type = VertexType::kFrontier;
        leaves.push_back(p);
        all_nodes.pop_back();
      }
      for (int i = 0; i < q.front()->children.size(); i++) q.push(q.front()->children[i]);
      q.pop();
      n--;
    }
  }
  // No clustering
  if (mb_params_.clustering == 0) {
    stat_->compute_exp_gain_time = GET_ELAPSED_TIME(tim);
    return leaves;
  }

  // FOR FULL PATH GAIN

  // Clustering:

  int clusters = 0;
  double prev_gain = 0.0;
  while (all_nodes.size() >= 1) {
    bool higher = false;
    kd_free(kd_tree_);
    kd_tree_ = kd_create(3);
    for (int j = 0; j < all_nodes.size(); j++) {
      kd_insert3(kd_tree_, all_nodes[j]->position(0), all_nodes[j]->position(1),
                 all_nodes[j]->position(2), (all_nodes[j]));
    }
    kdres* nearest =
        kd_nearest_range3(kd_tree_, all_nodes[0]->position(0), all_nodes[0]->position(1),
                          all_nodes[0]->position(2), mb_params_.kdtree_range);

    StateNode* s_test;
    s_test = (StateNode*)kd_res_item_data(nearest);

    VolumetricGain v_gain_req = computeIndividualExplorationGain((*s_test));
    double required_gain = v_gain_req.gain;
    if (required_gain >= prev_gain)
      higher = true;
    else
      higher = false;

    s_test->vol_gain = v_gain_req;
    if (s_test->vol_gain.is_frontier) s_test->type = VertexType::kFrontier;
    final_all_nodes.push_back((s_test));
    std::vector<StateNode*>::iterator it =
        std::find(all_nodes.begin(), all_nodes.end(), s_test);
    all_nodes.erase(it);
    while (kd_res_next(nearest) != 0) {
      s_test = (StateNode*)kd_res_item_data(nearest);
      s_test->vol_gain = v_gain_req;
      if (s_test->vol_gain.is_frontier) s_test->type = VertexType::kFrontier;
      final_all_nodes.push_back((s_test));
      std::vector<StateNode*>::iterator it =
          std::find(all_nodes.begin(), all_nodes.end(), (s_test));
      all_nodes.erase(it);
    }
    prev_gain = required_gain;

    kd_res_free(nearest);
    clusters++;
  }
  if (final_all_nodes.size() == 1) {
    ROS_WARN("Only one leaf: ");
    final_all_nodes[0]->printit();
  }
  stat_->compute_exp_gain_time = GET_ELAPSED_TIME(tim);
  return leaves;
}
std::vector<StateNode*> MBTree::getShortestPath(StateNode* leaf, bool full_path) {
  std::vector<StateNode*> path;
  path.push_back(leaf);
  // while(leaf->parent != NULL)
  while (leaf->parent != NULL) {
    if (full_path) {
      for (int i = leaf->minions.size() - 1; i >= 0; i--) {
        path.push_back(leaf->minions[i]);
      }
    }
    path.push_back(leaf->parent);
    leaf = leaf->parent;
  }
  return path;
}  // goal first start lasts

void MBTree::getFreeSpacePointCloud(std::string sensor_name, StateVec state,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZ> free_cloud;
  std::vector<Eigen::Vector3d> multiray_endpoints;
  Eigen::Vector3d state_vec(state[0], state[1], state[2]);
  StateVec state_ip;
  state_ip[0] = state[0];
  state_ip[1] = state[1];
  state_ip[2] = state[2];
  state_ip[3] = state[3];
  sensor_params_.sensor[sensor_name].getFrustumEndpoints(state_ip, multiray_endpoints);
  for (auto ep : multiray_endpoints) {
    Eigen::Vector3d ray = (ep - state_vec);
    double ray_length = ray.norm();
    double voxel_size = 0.1;
    bool hit = false;
    for (int i = 0; i < (int)(ray_length / voxel_size); i++) {
      Eigen::Vector3d p = i * voxel_size * ray + state_vec;
      explorer::MapManager::VoxelStatus voxel_state = map_manager_->getVoxelStatus(p);
      if (voxel_state == explorer::MapManager::VoxelStatus::kOccupied) {
        hit = true;
        break;
      }
    }
    if (!hit) {
      pcl::PointXYZ data;
      Eigen::Matrix3d rot_W2B;
      rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      Eigen::Matrix3d rot_B2W;
      rot_B2W = rot_W2B.inverse();
      Eigen::Vector3d origin(state[0], state[1], state[2]);
      Eigen::Vector3d epb = rot_B2W * (ep - origin);
      data.x = epb(0);
      data.y = epb(1);
      data.z = epb(2);
      cloud->points.push_back(data);
    }
  }
}

void MBTree::freePointCloudtimerCallback(const ros::TimerEvent& event) {
  StateVec state;
  state[0] = current_state_.position(0);
  state[1] = current_state_.position(1);
  state[2] = current_state_.position(2);
  state[3] = current_state_.yaw;

  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<Eigen::Vector3d> multiray_endpoints;
  for (auto sensor_name : free_frustum_params_.sensor_list) {
    free_frustum_params_.sensor[sensor_name].getFrustumEndpoints(state, multiray_endpoints);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_free_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    map_manager_->getFreeSpacePointCloud(multiray_endpoints, state, temp_free_cloud);
    *free_cloud += *temp_free_cloud;
  }
  sensor_msgs::PointCloud2 out_cloud;
  pcl::toROSMsg(*free_cloud.get(), out_cloud);
  out_cloud.header.frame_id = "/m100/base_link";
  out_cloud.header.stamp = ros::Time::now();
  free_cloud_pub_.publish(out_cloud);
}

void MBTree::timerCallback(const ros::TimerEvent& event) {
  // Re-initialize until get non-zero value.
  if (rostime_start_.toSec() == 0) rostime_start_ = ros::Time::now();

  if (!odometry_ready) {
    ROS_WARN("Planner is waiting for odometry");
    return;
  }

  if (first_odom) {
    first_odom = false;
    tree_root_->position = current_state_.position;
    StateVec root;
    root << tree_root_->position(0), tree_root_->position(1), tree_root_->position(2),
        tree_root_->yaw;
  }

  // Planning workspace visualizations
  StateVec root_state_vec(tree_root_->position(0), tree_root_->position(1),
                          tree_root_->position(2), tree_root_->yaw);
  visualizer_->visualizeWorkspace(root_state_vec, global_space_params_, local_space_params_);

  // Estimate the exploring direction.
  // Use the position direction rather than the heading direction.
  constexpr int kQueueMaxSize = 10;
  constexpr double kAlpha = 0.3;  // favor newest paths.
  constexpr double kMinDist = 0.5;
  if (robot_state_queue_.size()) {
    StateNode last_state = robot_state_queue_.back();
    Eigen::Vector3d cur_dir(current_state_.position(0) - last_state.position(0),
                            current_state_.position(1) - last_state.position(1),
                            0.0);  // ignore changes in z-axis

    if (cur_dir.norm() >= kMinDist) {
      double yaw = atan2(cur_dir[1], cur_dir[0]);
      double dyaw = yaw - exploring_direction_;
      truncateYaw(dyaw);
      exploring_direction_ = exploring_direction_ + (1 - kAlpha) * (dyaw);
      truncateYaw(exploring_direction_);
      if (robot_state_queue_.size() >= kQueueMaxSize) {
        robot_state_queue_.pop();
      }
      robot_state_queue_.emplace(current_state_);
    }

    visualizer_->visualizeRobotState(current_state_, robot_params_.size, exploring_direction_);
  } else {
    robot_state_queue_.emplace(current_state_);
  }
}

bool MBTree::compareAngles(double dir_angle_a, double dir_angle_b, double thres) {
  double dyaw = dir_angle_a - dir_angle_b;
  if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  else if (dyaw < -M_PI)
    dyaw += 2 * M_PI;

  if (std::abs(dyaw) <= thres) {
    return true;
  } else {
    return false;
  }
}

Eigen::Vector3d MBTree::estimateDirectionFromPath(std::vector<Eigen::Vector3d> path) {
  double k_alpha = 0.9;
  Eigen::Vector3d path_dir = Eigen::Vector3d(0.0, 0.0, 0.0);
  for (int i = 1; i < path.size(); i++) {
    Eigen::Vector3d curr_dir = Eigen::Vector3d(
        path[i](0) - path[i - 1](0), path[i](1) - path[i - 1](1), path[i](2) - path[i - 1](2));
    path_dir = path_dir + (1 - k_alpha) * (curr_dir - path_dir);
  }
  return path_dir;
}

std::vector<geometry_msgs::Pose> MBTree::getSafetyPath() {
  std::vector<geometry_msgs::Pose> safe_path;
  for (int i = 0; i < safe_path_.size(); i++) {
    geometry_msgs::Pose p;
    p.position.x = safe_path_[i]->position(0);
    p.position.y = safe_path_[i]->position(1);
    p.position.z = safe_path_[i]->position(2);
    // double yaw = atan2(safe_path_[i]->velocities(1),
    // safe_path_[i]->velocities(0));
    double yaw = current_state_.yaw;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    p.orientation.x = quat[0];
    p.orientation.y = quat[1];
    p.orientation.z = quat[2];
    p.orientation.w = quat[3];

    safe_path.push_back(p);
  }
  return safe_path;
}
std::vector<geometry_msgs::Pose> MBTree::getBestPath(int& status) {
  std::vector<StateNode *> path, f_path;
  bool stop = false;
  path.clear();
  prev_path_.clear();
  f_path = getShortestPath(best_vertex_, true);

  std::vector<geometry_msgs::Pose> final_path;
  std::vector<Eigen::Vector3d> test_path;  // Path to check the direction
  std::vector<geometry_msgs::Pose> full_path;
  final_path_.clear();

  for (int i = 0; i < f_path.size(); i++) {
    path.insert(path.begin(), f_path[i]);
  }
  double current_length = 0.0;
  int last_point = 0;
  for (int i = 0; i < path.size(); i++) {
    geometry_msgs::Pose p;
    p.position.x = path[i]->position(0);
    p.position.y = path[i]->position(1);
    p.position.z = path[i]->position(2);

    if (!stop) {
      final_path_.insert(final_path_.begin(), path[i]);
      test_path.push_back(path[i]->position);
      final_path.push_back(p);

      prev_path_.insert(prev_path_.begin(), path[i]);
      last_point++;
    }

    full_path.push_back(p);
    full_path_.insert(full_path_.begin(), path[i]);
    if (i > 0)
      current_length +=
          (test_path[test_path.size() - 1] - test_path[test_path.size() - 2]).norm();
    if (current_length >= mb_params_.max_traverse_length) stop = true;
  }
  double final_yaw = atan2(prev_path_[0]->velocities(1), prev_path_[0]->velocities(0));
  double current_yaw = current_state_.yaw;

  for (int i = 0; i < final_path.size(); i++) {
    double yaw;
    if (i > 0)
      yaw = atan2(final_path[i].position.y - final_path[i - 1].position.y,
                  final_path[i].position.x - final_path[i - 1].position.x);
    else {
      yaw = current_state_.yaw;
    }

    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    if (mb_params_.yaw_enable) {
      final_path[i].orientation.x = quat[0];
      final_path[i].orientation.y = quat[1];
      final_path[i].orientation.z = quat[2];
      final_path[i].orientation.w = quat[3];
    } else {
      final_path[i].orientation.x = 0.0;
      final_path[i].orientation.y = 0.0;
      final_path[i].orientation.z = 0.0;
      final_path[i].orientation.w = 1.0;
    }
  }
  for (int i = 0; i < full_path.size(); i++) {
    double yaw;
    if (i > 0)
      yaw = atan2(full_path[i].position.y - full_path[i - 1].position.y,
                  full_path[i].position.x - full_path[i - 1].position.x);
    else {
      yaw = current_state_.yaw;
    }

    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    if (mb_params_.yaw_enable) {
      full_path[i].orientation.x = quat[0];
      full_path[i].orientation.y = quat[1];
      full_path[i].orientation.z = quat[2];
      full_path[i].orientation.w = quat[3];
    } else {
      full_path[i].orientation.x = 0.0;
      full_path[i].orientation.y = 0.0;
      full_path[i].orientation.z = 0.0;
      full_path[i].orientation.w = 1.0;
    }
  }

  int test = predict_hit(final_path_[0]);
  std::vector<StateNode*> new_safe_path = final_path_[0]->safe_path;
  while (test < 0) {
    final_path_.insert(final_path_.begin(), full_path_[full_path_.size() - 1 - last_point]);
    test_path.push_back(full_path_[last_point]->position);
    prev_path_.insert(prev_path_.begin(), full_path_[full_path_.size() - 1 - last_point]);
    final_path.push_back(full_path[last_point]);

    test = predict_hit(final_path_[0]);
    new_safe_path = final_path_[0]->safe_path;
    last_point++;
  }
  safe_path_ = new_safe_path;

  constexpr double kDiffAngleForwardThres = 2 * M_PI / 3;

  double best_path_direction = explorer::Trajectory::estimateDirectionFromPath(test_path);
  bool result =
      compareAngles(exploring_direction_, best_path_direction, kDiffAngleForwardThres);
  if (!result)
    ROS_WARN("Changing exploration direction.[%f --> %f]", exploring_direction_,
             best_path_direction);

  bool check_all = false;
  int req_pt = 0;
  bool hit = false;

  visualizer_->visualizeMBPath(final_path, full_path, orig_path_);

  // Update total exploration path length:
  for (int i = 1; i < final_path.size(); i++) {
    double delta_path =
        (Eigen::Vector3d(final_path[i].position.x, final_path[i].position.y,
                         final_path[i].position.z) -
         Eigen::Vector3d(final_path[i - 1].position.x, final_path[i - 1].position.y,
                         final_path[i - 1].position.z))
            .norm();
    total_path_length_ += delta_path;
  }

  double curr_path_length = 0.0;
  for (int i = 1; i < full_path.size(); i++) {
    double delta_path =
        (Eigen::Vector3d(full_path[i].position.x, full_path[i].position.y,
                         full_path[i].position.z) -
         Eigen::Vector3d(full_path[i - 1].position.x, full_path[i - 1].position.y,
                         full_path[i - 1].position.z))
            .norm();
    curr_path_length += delta_path;
  }

  next_root_ = final_path_[0];
  root_storage.position = next_root_->position;
  root_storage.velocities = next_root_->velocities;
  next_vels_ = next_root_->velocities;
  status = 0;

  stat_->path_length = curr_path_length;

  return final_path;
}

std::vector<StateNode*> MBTree::get_prev_path() { return prev_path_; }

Eigen::Vector3d MBTree::getNextVel() { return next_vels_; }

void MBTree::printTimes() { stat_->printTime(); }

void MBTree::publishLogInfo() {
  planner_msgs::PlannerLogger log_msgs;

  log_msgs.header.stamp = ros::Time::now();
  ROS_INFO("Time Elapsed: %f", log_msgs.header.stamp.toSec());
  log_msgs.graph_build_time = stat_->build_tree_time;
  log_msgs.exp_gain_time = stat_->compute_exp_gain_time;
  log_msgs.total_time = stat_->total_time;
  log_msgs.path_length = stat_->path_length;

  log_msgs.find_frontier = 0.0;
  log_msgs.time_search_path_to_frontier = 0.0;

  log_msgs.map_resolution = map_manager_->getResolution();
#ifndef USE_OCTOMAP
  map_manager_->getNumVoxels(log_msgs.number_occupied_voxels, log_msgs.number_free_voxels);
#endif
  planner_logger_pub_.publish(log_msgs);
}

}  // mbplanner
}  // explorer