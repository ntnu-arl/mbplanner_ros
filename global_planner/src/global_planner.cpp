#include "global_planner/global_planner.h"
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

#define SQ(x) (x * x)

namespace explorer {

GlobalPlanner::GlobalPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  world_frame_id = "world";

  planner_trigger_time_ = 0;

#ifdef USE_OCTOMAP
  map_manager_ = new MapManagerOctomap(nh_, nh_private_);
#else
  map_manager_ =
      new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(nh_, nh_private_);
#endif

  visualization_ = new Visualization(nh_, nh_private_);

  // reset
  global_graph_.reset(new GraphManager());
  local_graph_.reset(new GraphManager());
  robot_state_hist_.reset(new RobotStateHistory());
  current_battery_time_remaining_ = std::numeric_limits<double>::max();
  stat_.reset(new SampleStatistic());

  // Load params:
  loadParams();

  // Subscribe to local planner tree
  local_tree_subscriber_ =
      nh_.subscribe("local_tree", 100, &GlobalPlanner::localTreeCallback, this);

  rostime_start_ = ros::Time::now();
  robot_backtracking_prev_ = NULL;
  odometry_ready = false;
  // Timer
  periodic_timer_ = nh_.createTimer(ros::Duration(0.25), &GlobalPlanner::timerCallback, this);
  global_graph_update_timer_ =
      nh_.createTimer(ros::Duration(kGlobalGraphUpdateTimerPeriod),
                      &GlobalPlanner::expandGlobalGraphTimerCallback, this);
}

void GlobalPlanner::localTreeCallback(const std_msgs::UInt8MultiArray& msg) {
  ros::Time ttime1;
  START_TIMER(ttime1);
  if (planner_trigger_time_ == 0) planner_trigger_time_++;
  local_graph_.reset(new GraphManager());
  Vertex* first_vertex = new Vertex(global_graph_->generateVertexID(), current_state_);
  first_vertex->type = VertexType::kUnvisited;
  global_graph_->addVertex(first_vertex);

  namespace ser = ros::serialization;

  std::vector<SerializeVertex> vertices_ip;
  uint32_t serial_size = ros::serialization::serializationLength(vertices_ip);
  std::vector<uint8_t> buffer;

  for (int i = 0; i < msg.data.size(); i++) buffer.push_back(msg.data[i]);
  ser::IStream streami(buffer.data(), buffer.size());
  ser::deserialize(streami, vertices_ip);

  START_TIMER(ttime1);
  std::vector<Vertex*> vertices_to_add;
  std::vector<Vertex*> frontier_vertices;
  const double kRangeCheck = 1.0;
  const double kUpdateRadius = 3.0;
  bool reset = true;
  for (int i = 0; i < vertices_ip.size(); i++) {
    StateVec st;
    st = vertices_ip[i].state;
    Vertex* ver = new Vertex(i + 1, st);  // temporary id to generate vertex list.
    ver->is_leaf_vertex = false;
    ver->vol_gain = vertices_ip[i].vol_gain;
    if (vertices_ip[i].type == 2)
      ver->type = VertexType::kFrontier;
    else if (vertices_ip[i].type == 1)
      ver->type = VertexType::kVisited;
    else
      ver->type = VertexType::kUnvisited;

    if (ver->type == VertexType::kFrontier) {
      Vertex* nearest_vertex = NULL;
      StateVec* nearest_state = NULL;
      if (global_graph_->getNearestVertexInRange(&(ver->state), kRangeCheck,
                                                 &nearest_vertex) ||
          robot_state_hist_->getNearestStateInRange(&(ver->state), kUpdateRadius,
                                                    &nearest_state)) {
        reset = true;
        continue;
      }
    }
    if (reset == false || ver->type == VertexType::kFrontier) {
      // Before adding the path, check if vertices already exist in that area.
      // Also, check if the robot has passed that are previously.
      // This will ensure that redundant vertices/paths are not added

      vertices_to_add.insert(vertices_to_add.begin(), ver);
      reset = false;
      if (vertices_ip[i].parent_id == -1) {
        addRefPathToGraph(global_graph_, vertices_to_add);
        vertices_to_add.clear();
        reset = true;
      }
    }
  }
  double graph_addition_time = GET_ELAPSED_TIME(ttime1);

  START_TIMER(ttime1);
  ROS_INFO("Global graph: %d vertices, %d edges.", global_graph_->getNumVertices(),
           global_graph_->getNumEdges());
  bool update_global_frontiers = true;
  if (update_global_frontiers) {
    std::vector<Vertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    ROS_INFO("Have %d frontiers from global graph.", (int)global_frontiers.size());
    for (auto& v : global_frontiers) {
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }
  double t_elapsed = GET_ELAPSED_TIME(ttime1);
  visualization_->visualizeGlobalGraph(global_graph_);
}

std::vector<int> GlobalPlanner::performShortestPathsClustering(
    const std::shared_ptr<GraphManager> graph_manager, const ShortestPathsReport& graph_rep,
    std::vector<Vertex*>& vertices, double dist_threshold, double principle_path_min_length,
    bool refinement_enable) {
  // Asumme long paths are principle paths.
  // Go over one by one starting from the longest ones.
  // Group them into clusters based on the normalized DTW distance metric.
  // Refine by choosing closest & valid principle path.

  // Sort into descending order.
  std::sort(vertices.begin(), vertices.end(),
            [&graph_manager, &graph_rep](const Vertex* a, const Vertex* b) {
              return graph_manager->getShortestDistance(a->id, graph_rep) >
                     graph_manager->getShortestDistance(b->id, graph_rep);
            });

  std::vector<std::vector<Eigen::Vector3d>> cluster_paths;
  std::vector<int> cluster_ids;
  for (int i = 0; i < vertices.size(); ++i) {
    std::vector<Eigen::Vector3d> path_cur;
    graph_manager->getShortestPath(vertices[i]->id, graph_rep, true, path_cur);
    bool found_a_neigbor = false;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      if (Trajectory::compareTwoTrajectories(path_cur, cluster_paths[j], dist_threshold)) {
        vertices[i]->cluster_id = cluster_ids[j];
        found_a_neigbor = true;
        break;
      }
    }
    if (!found_a_neigbor) {
      // Can not find any neigbor, set this as a new principle path.
      cluster_paths.emplace_back(path_cur);
      cluster_ids.push_back(vertices[i]->id);
      vertices[i]->cluster_id = vertices[i]->id;
    }
  }

  ROS_INFO("Cluster %d paths into %d clusters.", (int)vertices.size(),
           (int)cluster_paths.size());

  // Refinement step, remove short path and choose closest cluster.
  if (refinement_enable) {
    // Clean out noisy paths by removing short path.
    std::vector<std::vector<Eigen::Vector3d>> cluster_paths_refine;
    std::vector<int> cluster_ids_refine;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      double path_len = Trajectory::getPathLength(cluster_paths[j]);
      if (path_len >= principle_path_min_length) {
        cluster_paths_refine.push_back(cluster_paths[j]);
        cluster_ids_refine.push_back(cluster_ids[j]);
      }
    }
    // Recheck and choose closest one.
    int cluster_paths_size = cluster_paths.size();
    for (int i = 0; i < vertices.size(); ++i) {
      std::vector<Eigen::Vector3d> path_cur;
      graph_manager->getShortestPath(vertices[i]->id, graph_rep, true, path_cur);
      double dist_min = std::numeric_limits<double>::infinity();
      for (int j = 0; j < cluster_paths_refine.size(); ++j) {
        double dist_score = Trajectory::computeDistanceBetweenTwoTrajectories(
            path_cur, cluster_paths_refine[j]);
        if (dist_min > dist_score) {
          dist_min = dist_score;
          vertices[i]->cluster_id = cluster_ids_refine[j];
        }
      }
    }
    ROS_INFO("Clustering with refinement %d paths into %d clusters.", (int)vertices.size(),
             (int)cluster_paths_refine.size());
    return cluster_ids_refine;
  } else {
    return cluster_ids;
  }
}

bool GlobalPlanner::sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                                 StateVec& sampled_state) {
  bool found = false;
  int while_thres = 1000;  // Max sampling trials.
  while (!found && while_thres--) {
    random_sampler.generate(root_state, sampled_state);
    Eigen::Vector3d state(sampled_state[0], sampled_state[1], sampled_state[2]);
    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    if (sampled_state.x() + robot_params_.center_offset.x() <
        global_space_params_.min_val.x() + 0.5 * robot_box_size_.x()) {
      continue;
    } else if (sampled_state.y() + robot_params_.center_offset.y() <
               global_space_params_.min_val.y() + 0.5 * robot_box_size_.y()) {
      continue;
    } else if (sampled_state.z() + robot_params_.center_offset.z() <
               global_space_params_.min_val.z() + 0.5 * robot_box_size_.z()) {
      continue;
    } else if (sampled_state.x() + robot_params_.center_offset.x() >
               global_space_params_.max_val.x() - 0.5 * robot_box_size_.x()) {
      continue;
    } else if (sampled_state.y() + robot_params_.center_offset.y() >
               global_space_params_.max_val.y() - 0.5 * robot_box_size_.y()) {
      continue;
    } else if (sampled_state.z() + robot_params_.center_offset.z() >
               global_space_params_.max_val.z() - 0.5 * robot_box_size_.z()) {
      continue;
    }
    // Check if surrounding area is free.
    MapManager::VoxelStatus voxel_state = map_manager_->getBoxStatus(
        Eigen::Vector3d(sampled_state[0], sampled_state[1], sampled_state[2]) +
            robot_params_.center_offset,
        robot_box_size_, true);
    if (MapManager::VoxelStatus::kFree == voxel_state) {
      random_sampler.pushSample(sampled_state, true);  // for debug purpose.
      found = true;
    } else {
      stat_->num_vertices_fail++;
      random_sampler.pushSample(sampled_state, false);
    }
  }
  return found;
}

void GlobalPlanner::expandGraph(std::shared_ptr<GraphManager> graph_manager,
                                StateVec& new_state, ExpandGraphReport& rep,
                                bool allow_short_edge) {
  // Find nearest neighbour
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&new_state, &nearest_vertex)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  if (nearest_vertex == NULL) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  // Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();
  if (direction_norm > planning_params_.edge_length_max) {
    direction = planning_params_.edge_length_max * direction.normalized();
  } else if ((!allow_short_edge) && (direction_norm <= planning_params_.edge_length_min)) {
    // Should not add short edge.
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }
  // Recalculate the distance.
  direction_norm = direction.norm();
  new_state[0] = origin[0] + direction[0];
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];
  // Since we are buiding graph,
  // Consider to check the overshoot for both 2 directions except root node.
  Eigen::Vector3d overshoot_vec = planning_params_.edge_overshoot * direction.normalized();
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (nearest_vertex->id != 0) start_pos = start_pos - overshoot_vec;
  Eigen::Vector3d end_pos = origin + robot_params_.center_offset + direction + overshoot_vec;

  bool connected_to_root = false;
  if (MapManager::VoxelStatus::kFree ==
      map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true)) {
    Vertex* new_vertex = new Vertex(graph_manager->generateVertexID(), new_state);
    // Form a tree as the first step.
    if (nearest_vertex->id == 0) connected_to_root = true;
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex;
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    ++rep.num_edges_added;
    // Form more edges from neighbors if set RRG mode.
    if (planning_params_.rr_mode == explorer::RRModeType::kGraph) {
      std::vector<Vertex*> nearest_vertices;
      if (!graph_manager->getNearestVertices(&new_state, planning_params_.nearest_range,
                                             &nearest_vertices)) {
        rep.status = ExpandGraphStatus::kErrorKdTree;
        return;
      }
      origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
      for (int i = 0; i < nearest_vertices.size(); ++i) {
        if (nearest_vertices[i]->id == 0) connected_to_root = true;
        direction << nearest_vertices[i]->state[0] - origin[0],
            nearest_vertices[i]->state[1] - origin[1],
            nearest_vertices[i]->state[2] - origin[2];
        double d_norm = direction.norm();

        if ((d_norm > planning_params_.nearest_range_min) &&
            (d_norm < planning_params_.nearest_range_max)) {
          Eigen::Vector3d p_overshoot = direction / d_norm * planning_params_.edge_overshoot;
          Eigen::Vector3d p_start = origin + robot_params_.center_offset - p_overshoot;
          Eigen::Vector3d p_end = origin + robot_params_.center_offset + direction;
          if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;

          if (MapManager::VoxelStatus::kFree ==
              map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true)) {
            graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
            ++rep.num_edges_added;
          }
        }
      }
    }

  } else {
    stat_->num_edges_fail++;
    if (stat_->num_edges_fail < 500) {
      std::vector<double> vtmp = {start_pos[0], start_pos[1], start_pos[2],
                                  end_pos[0],   end_pos[1],   end_pos[2]};
      stat_->edges_fail.push_back(vtmp);
    }
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void GlobalPlanner::expandGraphEdges(std::shared_ptr<GraphManager> graph_manager,
                                     Vertex* new_vertex, ExpandGraphReport& rep) {
  std::vector<Vertex*> nearest_vertices;
  if (!graph_manager->getNearestVertices(&(new_vertex->state), planning_params_.nearest_range,
                                         &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  Eigen::Vector3d origin;
  origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
  for (int i = 0; i < nearest_vertices.size(); ++i) {
    Eigen::Vector3d direction;
    direction << nearest_vertices[i]->state[0] - origin[0],
        nearest_vertices[i]->state[1] - origin[1], nearest_vertices[i]->state[2] - origin[2];
    double d_norm = direction.norm();
    if ((d_norm > planning_params_.edge_length_min) &&
        (d_norm < planning_params_.edge_length_max)) {
      Eigen::Vector3d p_overshoot = direction / d_norm * planning_params_.edge_overshoot;
      Eigen::Vector3d p_start = origin + robot_params_.center_offset - p_overshoot;
      Eigen::Vector3d p_end = origin + robot_params_.center_offset + direction;
      if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;
      if (MapManager::VoxelStatus::kFree ==
          map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true)) {
        graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
        ++rep.num_edges_added;
      }
    }
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

bool GlobalPlanner::modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                               Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                               Eigen::Vector3d& p1_mod) {
  p1_mod = p1;

  Eigen::Vector3d p_center;
  p_center = (p0 + p1) / 2.0;
  Eigen::Vector3d p_dir;
  p_dir = (p1 - p0);
  double radius = p_dir.norm() / 2.0;
  Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  Eigen::Quaternion<double> quat_W2S;
  // quat_W2S.setFromTwoVectors(x_axis, p_dir.normalized()); // this creates an
  // ambiguity since it allows free rotations on 3-dof
  // Use the spherical with Cartesian (Forward, left, up) coordinate
  Eigen::Vector3d p_dir_norm = p_dir.normalized();
  double yaw_angle = std::atan2(p_dir_norm.y(), p_dir_norm.x());
  double pitch_angle = -std::atan2(p_dir_norm.z(), std::sqrt(p_dir_norm.x() * p_dir_norm.x() +
                                                             p_dir_norm.y() * p_dir_norm.y()));
  quat_W2S = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

  pcl::PointCloud<pcl::PointXYZ>* pcl_tf(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Translation<double, 3> trans_W2S(p_center);
  Eigen::Transform<double, 3, Eigen::Affine> tf_W2S(trans_W2S * quat_W2S);
  pcl::transformPointCloud(*obstacle_pcl, *pcl_tf, tf_W2S.inverse());

  // add a local bounding box
  double kDx = robot_params_.safety_extension[0];
  double kDy = robot_params_.safety_extension[1];
  double kDz = robot_params_.safety_extension[2];

  // 6 rectanges in form:  ax+by+cz = 1
  std::vector<Eigen::Vector3d> u_l;
  std::vector<Eigen::Vector3d> p_l;
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  p_l.push_back(Eigen::Vector3d(-radius - kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(radius + kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, -kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, -kDz));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, kDz));
  std::vector<Eigen::Vector3d> hyperplane_list;
  std::vector<Eigen::Vector3d> tangent_point_list;
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector3d a_l;
    a_l = u_l[i] / (u_l[i].dot(p_l[i]));
    tangent_point_list.push_back(p_l[i]);
    hyperplane_list.push_back(a_l);
  }

  // Keep points inside the local box only
  pcl::PointCloud<pcl::PointXYZ>* pcl_in_box(new pcl::PointCloud<pcl::PointXYZ>());
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    // Check all 6 hyperplanes
    const double kDSign = 0.05;  // numeric issue
    double sign;
    int i = 0;
    for (i = 0; i < 6; ++i) {
      sign = p->x * hyperplane_list[i].x() + p->y * hyperplane_list[i].y() +
             p->z * hyperplane_list[i].z() - 1;
      if (sign > kDSign) break;
    }
    if (i == 6) {
      // inside the local box
      pcl_in_box->push_back(*p);
    }
  }
  if (pcl_in_box->size())
    pcl::copyPointCloud(*pcl_in_box, *pcl_tf);
  else {
    // full free space --> keep current vertex.
    return true;
  }

  // Find closest point
  double dist_min_sq = std::numeric_limits<double>::max();
  Eigen::Vector3d p_tangent;
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    double dist_t = p->x * p->x + p->y * p->y + p->z * p->z;
    if (dist_t < dist_min_sq) {
      dist_min_sq = dist_t;
      p_tangent << p->x, p->y, p->z;
    }
  }

  const double kDDist = 0.01;  // deal with numeric error.
  if ((dist_min_sq == std::numeric_limits<double>::max()) || (dist_min_sq < kDDist)) {
    // the path is too close to obstacle.
    return false;
  }

  double a = radius, b = radius, c = radius;  // dimensions of the ellipsoid.
  // Check if we need to adjust the sphere to ellipsoid.
  if (dist_min_sq < (radius * radius)) {
    // Reduce other axes
    b = std::sqrt((p_tangent.y() * p_tangent.y() + p_tangent.z() * p_tangent.z()) /
                  (1 - p_tangent.x() * p_tangent.x() / (a * a)));
    c = b;  // Set equal b for now; but could increase.???
    // Fit the first hyperplane: x x_l + y y_l + z z_l = 1
    Eigen::Vector3d hyperplane_last = Eigen::Vector3d(
        p_tangent.x() / (a * a), p_tangent.y() / (b * b), p_tangent.z() / (c * c));
    hyperplane_list.push_back(hyperplane_last);
    tangent_point_list.push_back(p_tangent);
  }

  // Increase the ellipsoid and repeat.
  bool stop = false;
  int n_max = 0;  // magic number: max 50 hyperplanes
  while ((!stop) && (n_max < 50)) {
    ++n_max;
    pcl::PointCloud<pcl::PointXYZ>* pcl_reduced(new pcl::PointCloud<pcl::PointXYZ>());
    // Also re-scale each dimension followed the dimentions of ellipsoid
    if (hyperplane_list.size()) {
      Eigen::Vector3d hyperplane_last;
      hyperplane_last = hyperplane_list.back();
      // Reduce point: keep points on the same side with zero origin (sign < 0)
      for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
        double sign = p->x * hyperplane_last.x() + p->y * hyperplane_last.y() +
                      p->z * hyperplane_last.z() - 1;
        const double kDSign = -0.05;  // numeric issue
        if (sign < kDSign) {
          // same side with the ellipsoid.
          pcl_reduced->push_back(*p);
        }
      }
    } else {
      pcl::copyPointCloud(*pcl_tf, *pcl_reduced);
    }

    Eigen::Vector3d p_tangent1;
    dist_min_sq = std::numeric_limits<double>::max();
    for (auto p = pcl_reduced->begin(); p != pcl_reduced->end(); ++p) {
      // Scale to get next closest point.
      pcl::PointXYZ pv;
      pv.x = p->x / a;
      pv.y = p->y / b;
      pv.z = p->z / c;
      double dist_t = pv.x * pv.x + pv.y * pv.y + pv.z * pv.z;
      if (dist_t < dist_min_sq) {
        dist_min_sq = dist_t;
        p_tangent1 << p->x, p->y, p->z;
      }
    }
    if ((pcl_reduced->size() == 0) || (dist_min_sq == std::numeric_limits<double>::max())) {
      stop = true;
    } else {
      double e_ext = dist_min_sq;
      Eigen::Vector3d hyperplane_new =
          Eigen::Vector3d(p_tangent1.x() / (a * a * e_ext), p_tangent1.y() / (b * b * e_ext),
                          p_tangent1.z() / (c * c * e_ext));
      hyperplane_list.push_back(hyperplane_new);
      tangent_point_list.push_back(p_tangent1);
      pcl_tf->clear();
      pcl::copyPointCloud(*pcl_reduced, *pcl_tf);
    }
  }
  if (!stop) {
    // Require too many hyperplanes
    return false;
  }

  // Find the polygon formed from intersections between the bisector plane vs.
  // all hyperplanes
  // Not sure how to get the closed-form solution, also issue with unknown
  // voxels
  //  --> get average from uniform sampling on the y-z plane (body coordinate)
  std::vector<Eigen::Vector3d> feasible_samples;
  for (double dy = -kDy; dy < kDy; dy += 0.1) {
    for (double dz = -kDz; dz < kDz; dz += 0.1) {
      Eigen::Vector3d p(radius, dy, dz);
      // check if this is inside all hyperplanes.
      const double kDSign = -0.05;  // numeric issue
      double sign;
      int i = 0;
      for (i = 0; i < hyperplane_list.size(); ++i) {
        sign = p.x() * hyperplane_list[i].x() + p.y() * hyperplane_list[i].y() +
               p.z() * hyperplane_list[i].z() - 1;
        if (sign > kDSign) break;
      }
      if (i == hyperplane_list.size()) {
        feasible_samples.push_back(p);
      }
    }
  }

  for (int i = 0; i < hyperplane_list.size(); ++i) {
    tangent_point_list[i] = tf_W2S * tangent_point_list[i];  // convert back to world
    Eigen::Matrix4d tf_inv_T = tf_W2S.matrix().inverse().transpose();
    Eigen::Vector4d v_t;
    v_t = tf_inv_T * Eigen::Vector4d(hyperplane_list[i].x(), hyperplane_list[i].y(),
                                     hyperplane_list[i].z(), -1.0);
    v_t = v_t / (-v_t[3]);
    hyperplane_list[i] << v_t.x(), v_t.y(), v_t.z();
  }

  p1_mod << 0.0, 0.0, 0.0;
  int feasible_count = 0;
  for (int i = 0; i < feasible_samples.size(); ++i) {
    feasible_samples[i] = tf_W2S * feasible_samples[i];  // convert back to world
    // check if this is free voxel to deal with occluded area.
    if (map_manager_->getVoxelStatus(feasible_samples[i]) == MapManager::VoxelStatus::kFree) {
      feasible_corridor_pcl_->push_back(pcl::PointXYZ(
          feasible_samples[i].x(), feasible_samples[i].y(), feasible_samples[i].z()));
      p1_mod = p1_mod + feasible_samples[i];
      ++feasible_count;
    }
  }

  if (feasible_count) {
    p1_mod = p1_mod / feasible_count;
  } else {
    return false;
  }

  visualization_->visualizeHyperplanes(p_center, hyperplane_list, tangent_point_list);
  return true;
}

void GlobalPlanner::timerCallback(const ros::TimerEvent& event) {
  if (rostime_start_.toSec() == 0) rostime_start_ = ros::Time::now();

  // Enforce edges from odometry.
  bool enforce_vertex_from_odometry = true;
  constexpr double kOdoEnforceLength = 0.5;
  if (global_graph_->getNumVertices() > 0) {
    if (enforce_vertex_from_odometry) {
      while (robot_backtracking_queue_.size()) {
        StateVec bt_state = robot_backtracking_queue_.front();
        robot_backtracking_queue_.pop();
        if (robot_backtracking_prev_ == NULL)
          global_graph_->getNearestVertex(&bt_state, &robot_backtracking_prev_);

        if (robot_backtracking_prev_) {
          Eigen::Vector3d cur_dir(bt_state[0] - robot_backtracking_prev_->state[0],
                                  bt_state[1] - robot_backtracking_prev_->state[1],
                                  bt_state[2] - robot_backtracking_prev_->state[2]);
          double dir_norm = cur_dir.norm();
          if (dir_norm >= kOdoEnforceLength) {
            Vertex* new_vertex = new Vertex(global_graph_->generateVertexID(), bt_state);
            new_vertex->parent = robot_backtracking_prev_;
            new_vertex->distance = robot_backtracking_prev_->distance + dir_norm;
            global_graph_->addVertex(new_vertex);
            // could increase the distance to limit shortest paths along these
            // edges for
            // safety purpose since we don't check the collision
            const double kEdgeWeightExtended = 1.0;
            global_graph_->addEdge(new_vertex, robot_backtracking_prev_,
                                   dir_norm * kEdgeWeightExtended);
            robot_backtracking_prev_ = new_vertex;
          }
        }
      }
    }

    // Get position from odometry to add more vertices to the graph for homing.
    Eigen::Vector3d cur_dir(current_state_[0] - last_state_marker_[0],
                            current_state_[1] - last_state_marker_[1],
                            current_state_[2] - last_state_marker_[2]);

    constexpr double kOdoUpdateMinLength = kOdoEnforceLength;
    if (cur_dir.norm() >= kOdoUpdateMinLength) {
      // Add this to the global graph.
      const bool add_odometry_to_global_graph = true;
      if (add_odometry_to_global_graph) {
        StateVec new_state;
        new_state = current_state_;
        // offsetZAxis(new_state);
        ExpandGraphReport rep;
        expandGraph(global_graph_, new_state, rep);
        // ROS_INFO("From odometry, added %d vertices and %d edges",
        // rep.num_vertices_added, rep.num_edges_added);
      }
      last_state_marker_ = current_state_;
    }
  }
  visualization_->visualizeGlobalGraph(global_graph_);
}

void GlobalPlanner::expandGlobalGraphTimerCallback(const ros::TimerEvent& event) {
  // Algorithm:
  // Extract unvisited vertices in the global graph.
  // Randomly choose a vertex then group all nearby vertices within a local
  // bounding box.
  // Repeat again until having set of local bounding box covered all unvisited
  // vertices.
  // Random sample a collision free vertex inside a local box, expand the graph,
  // and compute the volumetric gain to check if this is frontier
  //

  if (planner_trigger_time_ == 0) return;  // Till odometry is not ready

  bool update_global_frontiers = false;
  if (update_global_frontiers) {
    std::vector<Vertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    for (auto& v : global_frontiers) {
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      // computeVolumetricGainRayModel(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }

  std::vector<Vertex*> unvisited_vertices;
  int global_graph_size = global_graph_->getNumVertices();
  for (int id = 0; id < global_graph_size; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kUnvisited) {
      unvisited_vertices.push_back(global_graph_->getVertex(id));
    }
  }
  if (unvisited_vertices.empty()) return;

  int num_unvisited_vertices = unvisited_vertices.size();
  const double kLocalBoxRadius = 10;
  const double kLocalBoxRadiusSq = kLocalBoxRadius * kLocalBoxRadius;
  std::vector<Eigen::Vector3d> cluster_centroids;
  std::vector<Vertex*> unvisited_vertices_remain;
  while (true) {
    unvisited_vertices_remain.clear();
    // Randomly pick a vertex
    int ind = rand() % (unvisited_vertices.size());
    // Find all vertices nearby this vertex.
    // Compute the centroid of this cluster.
    Eigen::Vector3d cluster_center(0, 0, 0);
    int num_vertices_in_cluster = 0;
    for (int i = 0; i < unvisited_vertices.size(); ++i) {
      Eigen::Vector3d dist(
          unvisited_vertices[i]->state.x() - unvisited_vertices[ind]->state.x(),
          unvisited_vertices[i]->state.y() - unvisited_vertices[ind]->state.y(),
          unvisited_vertices[i]->state.z() - unvisited_vertices[ind]->state.z());
      if (dist.squaredNorm() <= kLocalBoxRadiusSq) {
        cluster_center = cluster_center + Eigen::Vector3d(unvisited_vertices[i]->state.x(),
                                                          unvisited_vertices[i]->state.y(),
                                                          unvisited_vertices[i]->state.z());
        ++num_vertices_in_cluster;
      } else {
        unvisited_vertices_remain.push_back(unvisited_vertices[i]);
      }
    }
    cluster_center = cluster_center / num_vertices_in_cluster;
    cluster_centroids.push_back(cluster_center);
    unvisited_vertices = unvisited_vertices_remain;
    if (unvisited_vertices.empty()) break;
  }
  // ROS_WARN("Divided all unvisisted vertices into %d clusters.", cluster_centroids.size());

  // Expand global graph.
  // random_sampler_.reset();
  ros::Time time_lim;
  START_TIMER(time_lim);

  double time_elapsed = 0;
  int loop_count = 0, loop_count_success = 0;
  int num_vertices = 1;
  int num_edges = 0;
  while (time_elapsed < kGlobalGraphUpdateTimeBudget) {
    time_elapsed = GET_ELAPSED_TIME(time_lim);
    ++loop_count;
    for (int i = 0; i < cluster_centroids.size(); ++i) {
      StateVec centroid_state(cluster_centroids[i].x(), cluster_centroids[i].y(),
                              cluster_centroids[i].z(), 0);
      StateVec new_state;
      if (!sampleVertex(random_sampler_, centroid_state, new_state)) continue;
      // offsetZAxis(new_state, true);
      // Only expand samples in sparse areas & not yet passed by the robot & not
      // closed to any frontiers
      const double kSparseRadius = planning_params_.sparse_radius;              // m
      const double kOverlappedFrontierRadius = planning_params_.overlapped_frontier_radius;  // m
      std::vector<StateVec*> s_res;
      robot_state_hist_->getNearestStates(&new_state, kSparseRadius, &s_res);
      if (s_res.size()) continue;
      std::vector<Vertex*> v_res;
      global_graph_->getNearestVertices(&new_state, kSparseRadius, &v_res);
      if (v_res.size()) continue;
      std::vector<Vertex*> f_res;
      global_graph_->getNearestVertices(&new_state, kOverlappedFrontierRadius, &f_res);
      bool frontier_existed = false;
      for (auto v : f_res) {
        if (v->type == VertexType::kFrontier) {
          frontier_existed = true;
          break;
        }
      }
      if (frontier_existed) continue;
      loop_count_success++;
      ExpandGraphReport rep;
      expandGraph(global_graph_, new_state, rep);
      if (rep.status == ExpandGraphStatus::kSuccess) {
        computeVolumetricGainRayModel(rep.vertex_added->state, rep.vertex_added->vol_gain,
                                      false);
        if (rep.vertex_added->vol_gain.is_frontier)
          rep.vertex_added->type = VertexType::kFrontier;
        num_vertices += rep.num_vertices_added;
        num_edges += rep.num_edges_added;
      }
    }
  }

  time_elapsed = GET_ELAPSED_TIME(time_lim);
}

void GlobalPlanner::setPlanningTriggerTime() {
  ROS_INFO("Local planning started. Building Global Graph.");
  if (odometry_ready) {
    Vertex* new_vertex = new Vertex(global_graph_->generateVertexID(), current_state_);
    global_graph_->addVertex(new_vertex);
    planner_trigger_time_++;
  }
}

void GlobalPlanner::setState(StateVec& state) {
  odometry_ready = true;
  current_state_ = state;
  if (robot_backtracking_queue_.size()) {
    if (robot_backtracking_queue_.size() >= backtracking_queue_max_size) {
      robot_backtracking_queue_.pop();
    }
    robot_backtracking_queue_.emplace(current_state_);
  } else {
    robot_backtracking_queue_.emplace(state);
  }
}

bool GlobalPlanner::loadParams() {
  // Get the prefix name of the parameters.
  std::string ns = ros::this_node::getName();

  // Load all relevant parameters.
  if (!sensor_params_.loadParams(ns + "/SensorParams")) return false;
  if (!robot_params_.loadParams(ns + "/RobotParams")) return false;
  if (!global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global")) return false;
  if (!local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local")) return false;
  if (!planning_params_.loadParams(ns + "/PlanningParams")) return false;
  if (!random_sampler_.loadParams(ns + "/RandomSamplerParams/SamplerForExploration"))
    return false;
  if (!robot_dynamics_params_.loadParams(ns + "/RobotDynamics")) return false;

  // @todo A temporary solution to load the velocity setting.
  planning_params_.v_max = robot_dynamics_params_.v_max;
  planning_params_.v_homing_max = robot_dynamics_params_.v_homing_max;

  // All other relevant const values should be initialized in this call
  // after loading parameters for all fields.
  initializeParams();
  return true;
}

void GlobalPlanner::initializeParams() {
  // Compute constant values after loading all parameters to speed up
  // computation later.
  // Set sampler params from BoundedSpaceParams if required.
  random_sampler_.setParams(global_space_params_, local_space_params_);

  // Precompute the robot box for planning.
  robot_params_.getPlanningSize(robot_box_size_);
  planning_num_vertices_max_ = planning_params_.num_vertices_max;
  planning_num_edges_max_ = planning_params_.num_edges_max;

  // Get the global bounding box in the setting as default.
  // Visualize in the beginning for checking.
  global_bound_.setDefault(global_space_params_.min_val, global_space_params_.max_val);
}

void GlobalPlanner::computeVolumetricGain(StateVec& state, VolumetricGain& vgain,
                                          bool vis_en) {
  vgain.reset();
  double step_size = planning_params_.exp_gain_voxel_size;
  // Scan winthin a local space and sensor range.
  // Compute the local bound.
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;

  if (global_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = -global_space_params_.radius - global_space_params_.radius_extension;
      bound_max[i] = global_space_params_.radius + global_space_params_.radius_extension;
    }
  } else if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = global_space_params_.min_val[i] + global_space_params_.min_extension[i];
      bound_max[i] = global_space_params_.max_val[i] + global_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  gain_log.clear();
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  voxel_log.clear();
  // @TODO tung.
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];
    // Refine the bound within an effective range.
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i], state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Check all voxels inside local bound.
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d voxel;
    for (voxel[0] = bound_min[0]; voxel[0] < bound_max[0]; voxel[0] += step_size) {
      for (voxel[1] = bound_min[1]; voxel[1] < bound_max[1]; voxel[1] += step_size) {
        for (voxel[2] = bound_min[2]; voxel[2] < bound_max[2]; voxel[2] += step_size) {
          if (sensor_params_.sensor[sensor_name].isInsideFOV(state, voxel)) {
            MapManager::VoxelStatus vs_ray = map_manager_->getRayStatus(origin, voxel, true);
            if (vs_ray != MapManager::VoxelStatus::kOccupied) {
              MapManager::VoxelStatus vs = map_manager_->getVoxelStatus(voxel);
              if (vs == MapManager::VoxelStatus::kUnknown) {
                ++num_unknown_voxels;
              } else if (vs == MapManager::VoxelStatus::kFree) {
                ++num_free_voxels;
              } else if (vs == MapManager::VoxelStatus::kOccupied) {
                ++num_occupied_voxels;
              }
              if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
            }
          }
        }
      }
    }
    gain_log.push_back(
        std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels));
  }

  // Return gain values.
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }

  // Visualize if required.
  if (vis_en) {
    visualization_->visualizeVolumetricGain(bound_min, bound_max, voxel_log, step_size);
  }
}

void GlobalPlanner::computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                                  bool vis_en) {
  vgain.reset();

  // Scan winthin a local space and sensor range.
  // Compute the local bound.
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;

  if (global_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = -global_space_params_.radius - global_space_params_.radius_extension;
      bound_max[i] = global_space_params_.radius + global_space_params_.radius_extension;
    }
  } else if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = global_space_params_.min_val[i] + global_space_params_.min_extension[i];
      bound_max[i] = global_space_params_.max_val[i] + global_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  int raw_unk_voxels_count = 0;
  // @TODO tung.
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];
    // Refine the bound within an effective range.
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i], state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state, multiray_endpoints);
    map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp, voxel_log_tmp);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Have to remove those not belong to the local bound.
    // At the same time check if this is frontier.

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      MapManager::VoxelStatus vs = vl.second;
      if (vs == MapManager::VoxelStatus::kUnknown) ++raw_unk_voxels_count;
      int j = 0;
      for (j = 0; j < 3; j++) {
        if ((voxel[j] < bound_min[j]) || (voxel[j] > bound_max[j])) break;
      }
      if (j == 3) {
        // valid voxel.
        if (vs == MapManager::VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == MapManager::VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == MapManager::VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR("Unsupported voxel type.");
        }
        if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
      }
    }
    gain_log.push_back(
        std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels));
    if (vis_en) {
      visualization_->visualizeRays(state, multiray_endpoints);
    }

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
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }

// Visualize if required.
#if FULL_PLANNER_VIZ
  if (vis_en) {
    visualization_->visualizeVolumetricGain(bound_min, bound_max, voxel_log,
                                            map_manager_->getResolution());
  }
#endif
}

void GlobalPlanner::computeVolumetricGainRayModelNoBound(StateVec& state,
                                                         VolumetricGain& vgain) {
  vgain.reset();

  // Scan winthin a GLOBAL space and sensor range.
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;
  if (global_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = -global_space_params_.radius - global_space_params_.radius_extension;
      bound_max[i] = global_space_params_.radius + global_space_params_.radius_extension;
    }
  } else if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = global_space_params_.min_val[i] + global_space_params_.min_extension[i];
      bound_max[i] = global_space_params_.max_val[i] + global_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  // @TODO tung.
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];
    // Refine the bound within an effective range.
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i], state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state, multiray_endpoints);
    map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp, voxel_log_tmp);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Have to remove those not belong to the local bound.
    // At the same time check if this is frontier.

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      MapManager::VoxelStatus vs = vl.second;
      int j = 0;
      for (j = 0; j < 3; j++) {
        if ((voxel[j] < bound_min[j]) || (voxel[j] > bound_max[j])) break;
      }
      if (j == 3) {
        // valid voxel.
        if (vs == MapManager::VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == MapManager::VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == MapManager::VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR("Unsupported voxel type.");
        }
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
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }
}

bool GlobalPlanner::setHomingPos() {
  if (global_graph_->getNumVertices() == 0) {
    ROS_INFO("Global graph is empty: add current state as homing position.");
    Vertex* g_root_vertex = new Vertex(global_graph_->generateVertexID(), current_state_);
    global_graph_->addVertex(g_root_vertex);
    return true;
  } else {
    ROS_INFO("Global graph is not empty, can not set current state as homing.");
    return false;
  }
}

std::vector<geometry_msgs::Pose> GlobalPlanner::searchHomingPath(
    std::string tgt_frame, const StateVec& current_state) {
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN("[GlobalGraph] Graph is empty, nothing to search for homing.");
    return ret_path;
  }

  StateVec cur_state;
  cur_state << current_state[0], current_state[1], current_state[2], current_state[3];
  offsetZAxis(cur_state);
  Vertex* nearest_vertex = NULL;
  if (!global_graph_->getNearestVertex(&cur_state, &nearest_vertex)) return ret_path;
  if (nearest_vertex == NULL) return ret_path;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();

  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.0;
  const double kRadiusDelta = 0.2;
  bool connect_state_to_graph = true;
  if (direction_norm <= kRadiusLimit) {
    // Note: if kRadiusLimit <= edge_length_min it will fail with
    // kErrorShortEdge, dirty fix to check max
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph if small radius.
    Vertex* new_vertex = new Vertex(global_graph_->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    global_graph_->addVertex(new_vertex);
    global_graph_->addEdge(new_vertex, nearest_vertex, direction_norm);
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(global_graph_, new_vertex, rep);
    link_vertex = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(global_graph_, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      link_vertex = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      // Hopefully this one will not happen if the global planner always adds
      // vertices from odometry --> naive backtracking.
      connect_state_to_graph = false;
      ROS_WARN("[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("kErrorShortEdge.");
          break;
      }
      ROS_WARN("[GlobalGraph] Failed to find global path.");
    }
  }

  if (connect_state_to_graph) {
    if (!global_graph_->findShortestPaths(global_graph_rep_)) {
      ROS_ERROR("[GlobalGraph] Failed to find shortest path.");
      return ret_path;
    }
    std::vector<int> homing_path_id;
    global_graph_->getShortestPath(link_vertex->id, global_graph_rep_, false, homing_path_id);
    if (homing_path_id.empty() || homing_path_id.back() != 0) {
      ROS_ERROR("[GlobalGraph] Could not find a path to home.");
      return ret_path;
    }
    int homing_path_id_size = homing_path_id.size();
    for (int i = 0; i < homing_path_id_size; ++i) {
      StateVec state = global_graph_->getVertex(homing_path_id[i])->state;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, state[3]);
      tf::Vector3 origin(state[0], state[1], state[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      ret_path.push_back(pose);
    }

    // Set the heading angle tangent with the moving direction,
    // from the second waypoint; the first waypoint keeps the same direction.
    if (planning_params_.yaw_tangent_correction) {
      bool is_similar = comparePathWithDirectionApprioximately(
          ret_path, tf::getYaw(ret_path[0].orientation));
      for (int i = 0; i < (ret_path.size() - 1); ++i) {
        Eigen::Vector3d vec;
        if ((!planning_params_.homing_backward) || (is_similar)) {
          vec << ret_path[i + 1].position.x - ret_path[i].position.x,
              ret_path[i + 1].position.y - ret_path[i].position.y,
              ret_path[i + 1].position.z - ret_path[i].position.z;
        } else if (planning_params_.homing_backward) {
          vec << ret_path[i].position.x - ret_path[i + 1].position.x,
              ret_path[i].position.y - ret_path[i + 1].position.y,
              ret_path[i].position.z - ret_path[i + 1].position.z;
        }
        double yaw = std::atan2(vec[1], vec[0]);
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        ret_path[i + 1].orientation.x = quat.x();
        ret_path[i + 1].orientation.y = quat.y();
        ret_path[i + 1].orientation.z = quat.z();
        ret_path[i + 1].orientation.w = quat.w();
      }
    }
    visualization_->visualizeHomingPaths(global_graph_, global_graph_rep_, link_vertex->id);
  }
  visualization_->visualizeGlobalGraph(global_graph_);

  return ret_path;
}

std::vector<geometry_msgs::Pose> GlobalPlanner::getHomingPath(std::string tgt_frame) {
  // ros::Duration(3.0).sleep(); // sleep to unblock the thread to get and
  // update all latest pose update.
  // ros::spinOnce();

  std::vector<geometry_msgs::Pose> ret_path;
  ret_path = searchHomingPath(tgt_frame, current_state_);
  if (ret_path.size() < 1) return ret_path;

  // Re-assign yaw in beginning (HNI)
  double yaw = current_state_[3];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  ret_path[0].orientation.x = quat.x();
  ret_path[0].orientation.y = quat.y();
  ret_path[0].orientation.z = quat.z();
  ret_path[0].orientation.w = quat.w();

  // Modify the best path.
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path)) {
      ret_path = mod_path;
      // Re-assign yaw angle after modification.
      if (planning_params_.yaw_tangent_correction) {
        bool is_similar = comparePathWithDirectionApprioximately(
            ret_path, tf::getYaw(ret_path[0].orientation));
        for (int i = 0; i < (ret_path.size() - 1); ++i) {
          Eigen::Vector3d vec;
          if ((!planning_params_.homing_backward) || (is_similar)) {
            vec << ret_path[i + 1].position.x - ret_path[i].position.x,
                ret_path[i + 1].position.y - ret_path[i].position.y,
                ret_path[i + 1].position.z - ret_path[i].position.z;
          } else if (planning_params_.homing_backward) {
            vec << ret_path[i].position.x - ret_path[i + 1].position.x,
                ret_path[i].position.y - ret_path[i + 1].position.y,
                ret_path[i].position.z - ret_path[i + 1].position.z;
          }
          double yaw = std::atan2(vec[1], vec[0]);
          tf::Quaternion quat;
          quat.setEuler(0.0, 0.0, yaw);
          ret_path[i + 1].orientation.x = quat.x();
          ret_path[i + 1].orientation.y = quat.y();
          ret_path[i + 1].orientation.z = quat.z();
          ret_path[i + 1].orientation.w = quat.w();
        }
      }
    }

    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN("Compute an aternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  visualization_->visualizeRefPath(ret_path);
  return ret_path;

  // return searchHomingPath(tgt_frame, current_state_);
}

bool GlobalPlanner::isPathInCollision(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& end) const {
  MapManager::VoxelStatus v_status =
      map_manager_->getPathStatus(start, end, robot_box_size_, true);
  return (v_status != MapManager::VoxelStatus::kFree);
}

bool GlobalPlanner::isCollision(const Eigen::Vector3d& p) const {
  MapManager::VoxelStatus v_status = map_manager_->getBoxStatus(p, robot_box_size_, true);
  return (v_status != MapManager::VoxelStatus::kFree);
}

bool GlobalPlanner::improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                                    std::vector<geometry_msgs::Pose>& path_mod) {
  // Few heuristics to improve the path.
  // a) Shorten path by reducing intermidiate nodes. (be careful with turning
  // cases)
  // Shorten/reduce some very short paths to prevent small motion and sudden
  // change in angles.
  // b) Adjust nodes to its neighbors to improve safety
  // c) Ignore leaf node of the path to prevent the robot to come too close the
  // obstacle

  if (path_orig.empty()) return false;

  // Feature a) Remove short intermidiate vertices.
  std::vector<geometry_msgs::Pose> path_mod1 = path_orig;

  const double kSegmentLenMin = 0.5;
  bool cont_refine = true;
  while (cont_refine) {
    cont_refine = false;
    for (int i = 0; i < (path_mod1.size() - 2); ++i) {
      Eigen::Vector3d p_start(path_mod1[i].position.x, path_mod1[i].position.y,
                              path_mod1[i].position.z);
      Eigen::Vector3d p_int(path_mod1[i + 1].position.x, path_mod1[i + 1].position.y,
                            path_mod1[i + 1].position.z);
      Eigen::Vector3d p_end(path_mod1[i + 2].position.x, path_mod1[i + 2].position.y,
                            path_mod1[i + 2].position.z);
      Eigen::Vector3d segment = p_int - p_start;
      double segment_len = segment.norm();
      // ROS_WARN("Segment length %f.", segment_len);
      if (segment_len < kSegmentLenMin) {
        if ((MapManager::VoxelStatus::kFree ==
             map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true)) /*&&
            (!planning_params_.geofence_checking_enable ||
            (GeofenceManager::CoordinateStatus::kOK ==
             geofence_manager_->getPathStatus(Eigen::Vector2d(p_start[0], p_start[1]),
                                              Eigen::Vector2d(p_end[0], p_end[1]),
                                              Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))))*/) {
          // ignore the intermidiate nore, combine the first to the last node.
          ROS_WARN("Combine nodes to remove short segments.");
          path_mod1.erase(path_mod1.begin() + i + 1);
          cont_refine = true;
          break;
        }
      }
    }
  }

  // Implement (b) first: form a safe corridor along each path from cutting
  // hyperplanes.
  feasible_corridor_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  geometry_msgs::Pose pose0;
  pose0.position.x = path_mod1[0].position.x;
  pose0.position.y = path_mod1[0].position.y;
  pose0.position.z = path_mod1[0].position.z;
  pose0.orientation.x = path_mod1[0].orientation.x;
  pose0.orientation.y = path_mod1[0].orientation.y;
  pose0.orientation.z = path_mod1[0].orientation.z;
  pose0.orientation.w = path_mod1[0].orientation.w;
  path_mod.push_back(pose0);
  bool mod_success = true;
  for (int i = 1; i < path_mod1.size(); ++i) {
    Eigen::Vector3d p0(path_mod1[i - 1].position.x, path_mod1[i - 1].position.y,
                       path_mod1[i - 1].position.z);
    Eigen::Vector3d p0_mod(path_mod[i - 1].position.x, path_mod[i - 1].position.y,
                           path_mod[i - 1].position.z);
    Eigen::Vector3d p1(path_mod1[i].position.x, path_mod1[i].position.y,
                       path_mod1[i].position.z);
    Eigen::Vector3d p1_parallel = p0_mod + p1 - p0;

    Eigen::Vector3d p1_target;
    bool seg_free = true;
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getPathStatus(p0_mod, p1_parallel, robot_box_size_, true)) {
      p1_target = p1_parallel;
    } else if (MapManager::VoxelStatus::kFree ==
               map_manager_->getPathStatus(p0_mod, p1, robot_box_size_, true)) {
      p1_target = p1;
    } else {
      seg_free = false;
    }

    Eigen::Vector3d p1_mod;
    geometry_msgs::Pose pose;

    Eigen::Vector3d p_center;
    p_center = (p0_mod + p1_target) / 2.0;
    Eigen::Vector3d p_dir;
    p_dir = (p1 - p0);
    double radius = p_dir.norm() / 2.0;
    // add a local bounding box
    Eigen::Vector3d local_bbx(2 * (radius + robot_params_.safety_extension[0]),
                              2 * robot_params_.safety_extension[1],
                              2 * robot_params_.safety_extension[2]);
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::vector<Eigen::Vector3d> free_voxels;
    map_manager_->extractLocalMapAlongAxis(p_center, p_dir, local_bbx, occupied_voxels,
                                           free_voxels);

    pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto& v : occupied_voxels) {
      obstacle_pcl->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    }

    if (seg_free && (modifyPath(obstacle_pcl, p0_mod, p1_target, p1_mod))) {
      // Verify collision-free again with the map & geofence.
      if ((MapManager::VoxelStatus::kFree !=
           map_manager_->getPathStatus(p0_mod, p1_mod, robot_box_size_, true))) {
        p1_mod = p1;
        // mod_success = false;
        ROS_WARN("Newly modified path is not collision-free.");
        // break; // break to save time @recheck
      }
    } else {
      p1_mod = p1;
      // mod_success = false;
      // break; // break to save time @recheck
    }
    pose.position.x = p1_mod.x();
    pose.position.y = p1_mod.y();
    pose.position.z = p1_mod.z();
    path_mod.push_back(pose);
  }

  // Correct the heading angle tangent with the moving direction again.
  // Re-Assign the first heading
  path_mod[0].orientation.x = path_orig[0].orientation.x;
  path_mod[0].orientation.y = path_orig[0].orientation.y;
  path_mod[0].orientation.z = path_orig[0].orientation.z;
  path_mod[0].orientation.w = path_orig[0].orientation.w;
  if ((mod_success) && (planning_params_.yaw_tangent_correction)) {
    for (int i = 1; i < path_mod.size(); ++i) {
      Eigen::Vector3d vec(path_mod[i].position.x - path_mod[i - 1].position.x,
                          path_mod[i].position.y - path_mod[i - 1].position.y,
                          path_mod[i].position.z - path_mod[i - 1].position.z);
      if (planning_params_.planning_backward) vec = -vec;
      double yawhalf = 0.5 * std::atan2(vec[1], vec[0]);
      path_mod[i].orientation.x = 0.0;
      path_mod[i].orientation.y = 0.0;
      path_mod[i].orientation.z = sin(yawhalf);
      path_mod[i].orientation.w = cos(yawhalf);
    }
  }

  visualization_->visualizePCL(feasible_corridor_pcl_.get());
  return mod_success;
}

bool GlobalPlanner::addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                                      const std::vector<Vertex*>& vertices) {
  if (vertices.size() <= 0) return false;

  // The whole path is collision free already and start from root vertex.
  // We only need to link the first vertex to the existing graph.
  // Then add the whole path to the graph.
  // Finally, add more edges along the path to the graph.

  StateVec first_state;

  first_state << vertices[0]->state[0], vertices[0]->state[1], vertices[0]->state[2],
      vertices[0]->state[3];
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex)) return false;

  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0], first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  Vertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;

  // Add root vertex first.
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <= std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph.
    Vertex* new_vertex = new Vertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(graph_manager, first_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN("[GlobalGraph] Can not add current state to the global graph.");
      return false;
    }
  }

  // Add all remaining vertices of the path.
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (int i = 1; i < vertices.size(); ++i) {
    StateVec new_state;
    new_state << vertices[i]->state[0], vertices[i]->state[1], vertices[i]->state[2],
        vertices[i]->state[3];
    Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
                           parent_vertex->state[2]);
    Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                              new_state[2] - origin[2]);
    double direction_norm = direction.norm();
    Eigen::Vector3d new_state_vec3;
    new_state_vec3 << new_state(0), new_state(1), new_state(2);
    if (map_manager_->getPathStatus(new_state_vec3, origin, robot_params_.size, true) ==
        MapManager::VoxelStatus::kFree) {
      Vertex* new_vertex = new Vertex(graph_manager->generateVertexID(), new_state);
      new_vertex->type = vertices[i]->type;
      new_vertex->parent = parent_vertex;
      new_vertex->distance = parent_vertex->distance + direction_norm;
      parent_vertex->children.push_back(new_vertex);
      graph_manager->addVertex(new_vertex);
      graph_manager->addEdge(new_vertex, parent_vertex, direction_norm);
      vertex_list.push_back(new_vertex);
      parent_vertex = new_vertex;
    } else {
      break;
    }
  }

  // Build edges around vertices if possible to get better path.
  int n_vertices = 0;
  int n_edges = 0;
  // Assume the path is verified collision free.
  for (int i = 0; i < vertex_list.size(); ++i) {
    int num_vertices_added = 0;
    int num_edges_added = 0;
    ExpandGraphReport rep;
    expandGraphEdges(graph_manager, vertex_list[i], rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      n_vertices += num_vertices_added;
      n_edges += num_edges_added;
    } else {
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("Can not add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("Can not add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("Can not add this vertex: kErrorShortEdge.");
          break;
      }
    }
  }

  const bool path_intp_add = true;
  const double intp_len = 1.0;  // m
  if (path_intp_add) {
    // Add some intermidiate vertices along the path to densify the global
    // graph.
    for (int i = 0; i < (vertex_list.size() - 1); ++i) {
      Eigen::Vector3d start_vertex(vertex_list[i]->state.x(), vertex_list[i]->state.y(),
                                   vertex_list[i]->state.z());
      Eigen::Vector3d end_vertex(vertex_list[i + 1]->state.x(), vertex_list[i + 1]->state.y(),
                                 vertex_list[i + 1]->state.z());
      Eigen::Vector3d edge_vec = end_vertex - start_vertex;
      double edge_length = edge_vec.norm();
      if (edge_length <= intp_len) continue;
      edge_vec.normalize();
      int n_intp = (int)std::ceil(edge_length / intp_len);  // segments
      Vertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
        Vertex* new_vertex = new Vertex(graph_manager->generateVertexID(), new_state);
        graph_manager->addVertex(new_vertex);
        graph_manager->addEdge(new_vertex, prev_vertex, intp_len);
        prev_vertex = new_vertex;
        acc_len += intp_len;
      }
      // Link the last connection
      double last_edge_len = edge_length - acc_len;
      graph_manager->addEdge(prev_vertex, vertex_list[i + 1], last_edge_len);
    }
  }

  return true;
}

bool GlobalPlanner::addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                                      const std::vector<geometry_msgs::Pose>& path) {
  if (path.size() <= 0) return false;

  // The whole path is collision free already and start from root vertex.
  // We only need to link the first vertex to the existing graph.
  // Then add the whole path to the graph.
  // Finally, add more edges along the path to the graph.

  StateVec first_state;
  first_state << path[0].position.x, path[0].position.y, path[0].position.z, 0.0;
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex)) return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0], first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  Vertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;

  // Add root vertex first.
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <= std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph.
    Vertex* new_vertex = new Vertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(graph_manager, first_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN("[GlobalGraph] Can not add current state to the global graph.");
      return false;
    }
  }

  // Add all remaining vertices of the path.
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (int i = 1; i < path.size(); ++i) {
    StateVec new_state;
    new_state << path[i].position.x, path[i].position.y, path[i].position.z, 0.0;
    Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
                           parent_vertex->state[2]);
    Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                              new_state[2] - origin[2]);
    double direction_norm = direction.norm();

    Vertex* new_vertex = new Vertex(graph_manager->generateVertexID(), new_state);
    // new_vertex->type = vertices[i]->type;
    new_vertex->parent = parent_vertex;
    new_vertex->distance = parent_vertex->distance + direction_norm;
    parent_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, parent_vertex, direction_norm);
    vertex_list.push_back(new_vertex);
    parent_vertex = new_vertex;
  }

  // Build edges around vertices if possible to get better path.
  int n_vertices = 0;
  int n_edges = 0;
  // Assume the path is verified collision free.
  for (int i = 0; i < vertex_list.size(); ++i) {
    int num_vertices_added = 0;
    int num_edges_added = 0;
    ExpandGraphReport rep;
    expandGraphEdges(graph_manager, vertex_list[i], rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      n_vertices += num_vertices_added;
      n_edges += num_edges_added;
    } else {
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("Can not add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("Can not add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("Can not add this vertex: kErrorShortEdge.");
          break;
      }
    }
  }

  const bool path_intp_add = true;
  const double intp_len = 1.0;  // m
  if (path_intp_add) {
    // Add some intermidiate vertices along the path to densify the global
    // graph.
    for (int i = 0; i < (vertex_list.size() - 1); ++i) {
      Eigen::Vector3d start_vertex(vertex_list[i]->state.x(), vertex_list[i]->state.y(),
                                   vertex_list[i]->state.z());
      Eigen::Vector3d end_vertex(vertex_list[i + 1]->state.x(), vertex_list[i + 1]->state.y(),
                                 vertex_list[i + 1]->state.z());
      Eigen::Vector3d edge_vec = end_vertex - start_vertex;
      double edge_length = edge_vec.norm();
      if (edge_length <= intp_len) continue;
      edge_vec.normalize();
      int n_intp = (int)std::ceil(edge_length / intp_len);  // segments
      Vertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
        Vertex* new_vertex = new Vertex(graph_manager->generateVertexID(), new_state);
        graph_manager->addVertex(new_vertex);
        graph_manager->addEdge(new_vertex, prev_vertex, intp_len);
        prev_vertex = new_vertex;
        acc_len += intp_len;
      }
      // Link the last connection
      double last_edge_len = edge_length - acc_len;
      graph_manager->addEdge(prev_vertex, vertex_list[i + 1], last_edge_len);
    }
  }

  return true;
}

bool GlobalPlanner::comparePathWithDirectionApprioximately(
    const std::vector<geometry_msgs::Pose>& path, double yaw) {
  const double kMinSegmentLen = 2.0;
  const double kYawThres = 0.5 * M_PI;

  if (path.size() <= 1) return true;
  // Get aprpoximate direction.
  Eigen::Vector3d root_pos(path[0].position.x, path[0].position.y, path[0].position.z);
  double path_yaw = 0;
  for (int i = 1; i < path.size(); ++i) {
    Eigen::Vector3d dir_vec;
    dir_vec << path[i].position.x - root_pos.x(), path[i].position.y - root_pos.y(),
        0.0;  // ignore z
    if (dir_vec.norm() > kMinSegmentLen) {
      path_yaw = std::atan2(dir_vec.y(), dir_vec.x());
      break;
    }
  }

  double dyaw = path_yaw - yaw;
  if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  else if (dyaw < -M_PI)
    dyaw += 2 * M_PI;

  const double yaw_thres = 0.5 * M_PI;
  if (std::abs(dyaw) <= kYawThres) {
    return true;
  } else {
    return false;
  }
}

MapManager::VoxelStatus GlobalPlanner::getPathStatus(const Eigen::Vector3d& start,
                                                     const Eigen::Vector3d& end,
                                                     bool stop_at_unknown_voxel) const {
  return map_manager_->getPathStatus(start, end, robot_params_.size, stop_at_unknown_voxel);
}

MapManager::VoxelStatus GlobalPlanner::getBoxStatus(const Eigen::Vector3d& p,
                                                    bool stop_at_unknown_voxel) const {
  return map_manager_->getBoxStatus(p, robot_params_.size, stop_at_unknown_voxel);
}

std::vector<geometry_msgs::Pose> GlobalPlanner::runGlobalPlanner(int vertex_id) {
  ros::Duration(1.0).sleep();  // sleep to unblock the thread to get and update
                               // all latest pose update.
  ros::spinOnce();

  START_TIMER(ttime);
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN("[GlobalGraph] Graph is empty, nothing to search.");
    return ret_path;
  }

  // Get list of current frontiers.
  std::vector<Vertex*> global_frontiers;
  int num_vertices = global_graph_->getNumVertices();
  for (int id = 0; id < num_vertices; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
      global_frontiers.push_back(global_graph_->getVertex(id));
    }
  }
  ROS_INFO("Have %d frontiers from global graph.", (int)global_frontiers.size());
  if (global_frontiers.size() <= 0) {
    ROS_INFO("No frontier exists --> Call HOMING instead if fully explored.");
    return ret_path;
  }

  ROS_INFO("Re-check all frontiers.");
  global_frontiers.clear();
  for (int id = 0; id < num_vertices; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
      Vertex* v = global_graph_->getVertex(id);
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier)
        v->type = VertexType::kUnvisited;
      else
        global_frontiers.push_back(global_graph_->getVertex(id));
    }
  }
  ROS_INFO("After-checking: %d valid frontiers left.", (int)global_frontiers.size());

  // First try to add current state to the global graph.
  ROS_WARN("Trying to add new vertex from current position.");
  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2], current_state_[3];
  offsetZAxis(cur_state);
  Vertex* nearest_vertex = NULL;
  if (!global_graph_->getNearestVertex(&cur_state, &nearest_vertex)) return ret_path;
  if (nearest_vertex == NULL) return ret_path;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();
  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.5;  // 0.5

  bool connect_state_to_graph = true;
  if (direction_norm == 0) {
    // link_vertex = nearest_vertex;
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(global_graph_, nearest_vertex, rep);
    link_vertex = nearest_vertex;
  } else if (direction_norm <= std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    // Blindly add a link/vertex to the graph if small radius.
    Vertex* new_vertex = new Vertex(global_graph_->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    global_graph_->addVertex(new_vertex);
    global_graph_->addEdge(new_vertex, nearest_vertex, direction_norm);
    // link_vertex = new_vertex;
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(global_graph_, new_vertex, rep);
    link_vertex = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(global_graph_, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      link_vertex = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      connect_state_to_graph = false;
      ROS_WARN("[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("kErrorShortEdge.");
          break;
      }
    }
  }

  if (connect_state_to_graph) {
    ROS_WARN("Added current state to the graph. Start searching now.");
    // Get Dijsktra path from home to all.
    if (!global_graph_->findShortestPaths(global_graph_rep_)) {
      ROS_ERROR("[GlobalGraph] Failed to find shortest path.");
      return ret_path;
    }
    // Get Dijsktra path from current to all.
    ShortestPathsReport frontier_graph_rep;
    if (!global_graph_->findShortestPaths(link_vertex->id, frontier_graph_rep)) {
      ROS_ERROR("[GlobalGraph] Failed to find shortest path.");
      return ret_path;
    }
    // Get list of feasible frontiers by checking remaining time.
    // Leave the check empty for now since it relate to time budget setting.
    std::vector<Vertex*> feasible_global_frontiers;
    for (auto& f : global_frontiers) {
      feasible_global_frontiers.push_back(f);
    }
    ROS_INFO("Get %d feasible frontiers from global frontiers.",
             (int)feasible_global_frontiers.size());
    if (feasible_global_frontiers.size() <= 0) {
      ROS_INFO(
          "No feasible frontier exists --> Call HOMING instead if fully "
          "explored.");
      return ret_path;
    }

    // Must recompute exploration gain.
    double best_gain = -1.0;
    Vertex* best_frontier = NULL;
    std::unordered_map<int, double> frontier_exp_gain;
    for (int i = 0; i < feasible_global_frontiers.size(); ++i) {
      Vertex* f = feasible_global_frontiers[i];
      // get gain.
      std::vector<int> current_to_frontier_path_id;
      std::vector<int> frontier_to_home_path_id;
      double current_to_frontier_distance;
      double frontier_to_home_distance;

      global_graph_->getShortestPath(f->id, frontier_graph_rep, true,
                                     current_to_frontier_path_id);
      global_graph_->getShortestPath(f->id, global_graph_rep_, false,
                                     frontier_to_home_path_id);
      current_to_frontier_distance =
          global_graph_->getShortestDistance(f->id, frontier_graph_rep);
      frontier_to_home_distance = global_graph_->getShortestDistance(f->id, global_graph_rep_);
      const double g_distance_penalty = 0.01;
      double exp_gain =
          f->vol_gain.gain * exp(-g_distance_penalty * current_to_frontier_distance);
      frontier_exp_gain[f->id] = exp_gain;
      if (exp_gain > best_gain) {
        best_gain = exp_gain;
        best_frontier = f;
      }
    }

    // Rank from the best one.
    // Sort into descending order.
    std::sort(feasible_global_frontiers.begin(), feasible_global_frontiers.end(),
              [&frontier_exp_gain](const Vertex* a, const Vertex* b) {
                return frontier_exp_gain[a->id] > frontier_exp_gain[b->id];
              });
    // Print out
    // ROS_WARN("List of potential frontier in decreasing order of gain:");
    // for (int i = 0; i < feasible_global_frontiers.size(); ++i) {
    //   ROS_INFO("ID [%d]: %d with gain %f", i,
    //   feasible_global_frontiers[i]->id,
    //   frontier_exp_gain[feasible_global_frontiers[i]->id]);
    // }

    if (vertex_id == 0) {
      // select the best one.
      best_frontier = feasible_global_frontiers[0];
      best_gain = frontier_exp_gain[best_frontier->id];
    } else {
      // id of the node.
      bool found_id = false;
      for (int ii = 0; ii < feasible_global_frontiers.size(); ++ii) {
        if (feasible_global_frontiers[ii]->id == vertex_id) {
          best_frontier = feasible_global_frontiers[ii];
          best_gain = frontier_exp_gain[best_frontier->id];
          found_id = true;
          break;
        }
      }
      if (!found_id) {
        ROS_WARN("Vertex with ID [%d] is not frontier. Please select another ID.", vertex_id);
        visualization_->visualizeGlobalGraph(global_graph_);
        return ret_path;
      }
    }

    std::vector<int> current_to_frontier_path_id;
    std::vector<int> frontier_to_home_path_id;
    if (best_gain >= 0) {
      ROS_WARN("Found the best frontier to go is: %d", best_frontier->id);

      global_graph_->getShortestPath(best_frontier->id, frontier_graph_rep, true,
                                     current_to_frontier_path_id);
      global_graph_->getShortestPath(best_frontier->id, global_graph_rep_, false,
                                     frontier_to_home_path_id);
      int current_to_frontier_path_id_size = current_to_frontier_path_id.size();
      for (int i = 0; i < current_to_frontier_path_id_size; ++i) {
        StateVec state = global_graph_->getVertex(current_to_frontier_path_id[i])->state;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, state[3]);
        tf::Vector3 origin(state[0], state[1], state[2]);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        ret_path.push_back(pose);
      }
    } else {
      ROS_WARN(
          "Could not find any positive gain (Should not happen) --> Try with "
          "HOMING.");
      return ret_path;
    }

    // Set the heading angle tangent with the moving direction,
    // from the second waypoint; the first waypoint keeps the same direction.
    if (planning_params_.yaw_tangent_correction) {
      for (int i = 0; i < (ret_path.size() - 1); ++i) {
        Eigen::Vector3d vec(ret_path[i + 1].position.x - ret_path[i].position.x,
                            ret_path[i + 1].position.y - ret_path[i].position.y,
                            ret_path[i + 1].position.z - ret_path[i].position.z);
        double yaw = std::atan2(vec[1], vec[0]);
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        ret_path[i + 1].orientation.x = quat.x();
        ret_path[i + 1].orientation.y = quat.y();
        ret_path[i + 1].orientation.z = quat.z();
        ret_path[i + 1].orientation.w = quat.w();
      }
    }

    // Modify path if required
    if (planning_params_.path_safety_enhance_enable) {
      ros::Time mod_time;
      START_TIMER(mod_time);
      std::vector<geometry_msgs::Pose> mod_path;
      if (improveFreePath(ret_path, mod_path)) {
        ret_path = mod_path;
      }
      double dmod_time = GET_ELAPSED_TIME(mod_time);
      ROS_WARN("Compute an aternate path for homing in %f(s)", dmod_time);
      visualization_->visualizeModPath(mod_path);
    }

    visualization_->visualizeGlobalPaths(global_graph_, current_to_frontier_path_id,
                                         frontier_to_home_path_id);
  }
  double dtime = GET_ELAPSED_TIME(ttime);
  ROS_WARN("runGlobalPlanner costs: %f (s)", dtime);

  visualization_->visualizeGlobalGraph(global_graph_);
  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}

RobotStateHistory::RobotStateHistory() {
  kd_tree_ = NULL;
  reset();
}

void RobotStateHistory::reset() {
  // Reset kdtree first.
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
}

void RobotStateHistory::addState(StateVec* s) {
  kd_insert3(kd_tree_, s->x(), s->y(), s->z(), s);
  state_hist_.push_back(s);
}

bool RobotStateHistory::getNearestState(const StateVec* state, StateVec** s_res) {
  // it seems kdtree lib  can not deal with empty tree, put a guard check here.
  if (state_hist_.size() == 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *s_res = (StateVec*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool RobotStateHistory::getNearestStates(const StateVec* state, double range,
                                         std::vector<StateVec*>* s_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  if (state_hist_.size() == 0) return false;
  kdres* neighbors = kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  s_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    StateVec* new_neighbor = (StateVec*)kd_res_item_data(neighbors);
    s_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool RobotStateHistory::getNearestStateInRange(const StateVec* state, double range,
                                               StateVec** s_res) {
  if (state_hist_.size() == 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *s_res = (StateVec*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*s_res)->x(), state->y() - (*s_res)->y(), state->z() - (*s_res)->z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}
}