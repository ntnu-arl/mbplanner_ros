#include "planner_common/visualizer.h"
#include <tf/transform_datatypes.h>

namespace explorer {

Visualization::Visualization(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) {
  nh_ = nh;
  nh_private_ = nh_private;
  planning_workspace_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_workspace", 10);
  sensor_fov_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/sensor_fov", 100);

  planning_global_graph_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_global_graph", 10);
  planning_homing_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_homing_path", 10);
  planning_global_path_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_global_path", 10);
  ref_paths_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/ref_path", 10);
  volumetric_gain_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/volumetric_gains", 10);
  rays_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/ray_casting", 10);
  hyperplanes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/hyperplanes", 10);
  mod_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/mod_path", 10);
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("vis/occupied_pcl", 10);

  viz_final_path_pub_ = nh_.advertise<nav_msgs::Path>("vis/local_planning_final_path", 10);
  viz_full_path_pub_ = nh_.advertise<nav_msgs::Path>("vis/local_planning_full_path", 10);
  viz_original_path_pub_ =
      nh_.advertise<nav_msgs::Path>("vis/local_planning_original_path", 10);
  viz_tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_mb_tree", 10);
  safety_paths_pub_ = nh_.advertise<visualization_msgs::Marker>("vis/safety_paths", 10);
  viz_gains_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/gains", 10);
  similar_paths_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_similar_paths", 10);
  robot_box_pub_ =
      nh_.advertise<visualization_msgs::Marker>("vis/planning_robot_state_box", 10);
  exploration_direction_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("vis/planning_exploration_dir", 10);

  best_path_id_ = 0;
}

void Visualization::visualizeGlobalGraph(const std::shared_ptr<GraphManager> graph_manager) {
  std::shared_ptr<Graph> g = graph_manager->graph_;
  std::unordered_map<int, Vertex *> &v_map = graph_manager->vertices_map_;

  if (graph_manager->getNumVertices() == 0) return;
  if (planning_global_graph_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all edges using line (fast)
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.1;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator> ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 53.0 / 255.0;
  vertex_marker.color.g = 49.0 / 255.0;
  vertex_marker.color.b = 119.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  visualization_msgs::Marker frontier_marker;
  frontier_marker.header.stamp = ros::Time::now();
  frontier_marker.header.seq = 0;
  frontier_marker.header.frame_id = world_frame_id;
  frontier_marker.id = 0;
  frontier_marker.ns = "frontier";
  frontier_marker.action = visualization_msgs::Marker::ADD;
  frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_marker.scale.x = 0.5;
  frontier_marker.scale.y = 0.5;
  frontier_marker.scale.z = 0.5;
  frontier_marker.color.r = 1.0;
  frontier_marker.color.g = 0.0;
  frontier_marker.color.b = 0.0;
  frontier_marker.color.a = 1.0;
  frontier_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  frontier_marker.frame_locked = false;
  int num_vertices = graph_manager->getNumVertices();
  int marker_id = 0;
  for (int id = 0; id < num_vertices; ++id) {
    if (v_map[id]->type == VertexType::kFrontier) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      frontier_marker.points.push_back(p1);

      // Gain
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.seq = 0;
      marker.header.frame_id = world_frame_id;
      marker.ns = "gain";
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 2.0;  // text height
      marker.color.r = 18.0 / 255.0;
      marker.color.g = 15.0 / 255.0;
      marker.color.b = 196.0 / 255.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(shortest_paths_lifetime);
      marker.frame_locked = false;
      marker.pose.position.x = v_map[id]->state[0];
      marker.pose.position.y = v_map[id]->state[1];
      marker.pose.position.z = v_map[id]->state[2] + 0.1;
      std::string text_display = std::to_string(v_map[id]->id);

      marker.text = text_display;
      marker.id = marker_id++;
      marker_array.markers.push_back(marker);
    }
  }
  marker_array.markers.push_back(frontier_marker);
  planning_global_graph_pub_.publish(marker_array);
}

void Visualization::visualizeWorkspace(StateVec &state, BoundedSpaceParams &global_ws,
                                       BoundedSpaceParams &local_ws) {
  if (planning_workspace_pub_.getNumSubscribers() < 1) return;
  visualization_msgs::MarkerArray marker_array;

  // Global ws
  visualization_msgs::Marker global_ws_marker;
  global_ws_marker.header.stamp = ros::Time::now();
  global_ws_marker.header.seq = 0;
  global_ws_marker.header.frame_id = world_frame_id;
  global_ws_marker.id = 0;
  global_ws_marker.ns = "global";
  global_ws_marker.action = visualization_msgs::Marker::ADD;
  if (global_ws.type == BoundedSpaceType::kCuboid) {
    global_ws_marker.type = visualization_msgs::Marker::CUBE;
    global_ws_marker.pose.position.x = 0.5 * (global_ws.min_val[0] + global_ws.max_val[0]);
    global_ws_marker.pose.position.y = 0.5 * (global_ws.min_val[1] + global_ws.max_val[1]);
    global_ws_marker.pose.position.z = 0.5 * (global_ws.min_val[2] + global_ws.max_val[2]);
    global_ws_marker.scale.x = global_ws.max_val[0] - global_ws.min_val[0];
    global_ws_marker.scale.y = global_ws.max_val[1] - global_ws.min_val[1];
    global_ws_marker.scale.z = global_ws.max_val[2] - global_ws.min_val[2];
  } else if (global_ws.type == BoundedSpaceType::kSphere) {
    global_ws_marker.type = visualization_msgs::Marker::SPHERE;
    global_ws_marker.pose.position.x = 0;
    global_ws_marker.pose.position.y = 0;
    global_ws_marker.pose.position.z = 0;
    global_ws_marker.scale.x = global_ws.radius * 2;
    global_ws_marker.scale.y = global_ws.radius * 2;
    global_ws_marker.scale.z = global_ws.radius * 2;
  } else {
    ROS_WARN("Not supported.");
    return;
  }
  Eigen::Matrix3d g_rot;
  g_rot = Eigen::AngleAxisd(global_ws.rotations[0], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(global_ws.rotations[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(global_ws.rotations[2], Eigen::Vector3d::UnitX());
  Eigen::Quaterniond g_quat(g_rot);
  global_ws_marker.pose.orientation.x = g_quat.x();
  global_ws_marker.pose.orientation.y = g_quat.y();
  global_ws_marker.pose.orientation.z = g_quat.z();
  global_ws_marker.pose.orientation.w = g_quat.w();
  global_ws_marker.color.r = 200.0 / 255.0;
  global_ws_marker.color.g = 100.0 / 255.0;
  global_ws_marker.color.b = 0.0;
  global_ws_marker.color.a = 0.25;
  global_ws_marker.lifetime = ros::Duration(ws_lifetime);
  global_ws_marker.frame_locked = false;
  marker_array.markers.push_back(global_ws_marker);

  // Local ws
  visualization_msgs::Marker local_ws_marker;
  local_ws_marker.header.stamp = ros::Time::now();
  local_ws_marker.header.seq = 0;
  local_ws_marker.header.frame_id = world_frame_id;
  local_ws_marker.ns = "local";
  local_ws_marker.id = 0;
  local_ws_marker.action = visualization_msgs::Marker::ADD;
  if (local_ws.type == BoundedSpaceType::kCuboid) {
    local_ws_marker.type = visualization_msgs::Marker::CUBE;
    local_ws_marker.pose.position.x =
        state[0] + 0.5 * (local_ws.min_val[0] + local_ws.max_val[0]);
    local_ws_marker.pose.position.y =
        state[1] + 0.5 * (local_ws.min_val[1] + local_ws.max_val[1]);
    local_ws_marker.pose.position.z =
        state[2] + 0.5 * (local_ws.min_val[2] + local_ws.max_val[2]);
    local_ws_marker.scale.x = local_ws.max_val[0] - local_ws.min_val[0];
    local_ws_marker.scale.y = local_ws.max_val[1] - local_ws.min_val[1];
    local_ws_marker.scale.z = local_ws.max_val[2] - local_ws.min_val[2];
  } else if (local_ws.type == BoundedSpaceType::kSphere) {
    local_ws_marker.type = visualization_msgs::Marker::SPHERE;
    local_ws_marker.pose.position.x = state[0];
    local_ws_marker.pose.position.y = state[1];
    local_ws_marker.pose.position.z = state[2];
    local_ws_marker.scale.x = local_ws.radius * 2;
    local_ws_marker.scale.y = local_ws.radius * 2;
    local_ws_marker.scale.z = local_ws.radius * 2;
  } else {
    ROS_WARN("Not supported.");
    return;
  }

  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(local_ws.rotations[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(local_ws.rotations[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(local_ws.rotations[2], Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quatd(rot);
  local_ws_marker.pose.orientation.x = quatd.x();
  local_ws_marker.pose.orientation.y = quatd.y();
  local_ws_marker.pose.orientation.z = quatd.z();
  local_ws_marker.pose.orientation.w = quatd.w();
  local_ws_marker.color.r = 255.0 / 255.0;
  local_ws_marker.color.g = 100.0 / 255.0;
  local_ws_marker.color.b = 255.0 / 255.0;
  local_ws_marker.color.a = 0.25;
  local_ws_marker.lifetime = ros::Duration(ws_lifetime);
  local_ws_marker.frame_locked = false;
  marker_array.markers.push_back(local_ws_marker);

  planning_workspace_pub_.publish(marker_array);
}

void Visualization::visualizeHomingPaths(const std::shared_ptr<GraphManager> graph_manager,
                                         const ShortestPathsReport &graph_rep,
                                         int current_id) {
  if (planning_homing_pub_.getNumSubscribers() < 1) return;

  std::unordered_map<int, Vertex *> &v_map = graph_manager->vertices_map_;

  visualization_msgs::MarkerArray marker_array;

  // Plot the best one first.
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "best_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.4;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  bool cont = true;
  std::vector<int> id_best_path;
  graph_manager->getShortestPath(current_id, graph_rep, false, id_best_path);
  if (id_best_path.size() > 1) {
    for (int i = 0; i < (id_best_path.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[id_best_path[i]]->state[0];
      p1.y = v_map[id_best_path[i]]->state[1];
      p1.z = v_map[id_best_path[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[id_best_path[i + 1]]->state[0];
      p2.y = v_map[id_best_path[i + 1]]->state[1];
      p2.z = v_map[id_best_path[i + 1]]->state[2];
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);
  planning_homing_pub_.publish(marker_array);
}

void Visualization::visualizeGlobalPaths(const std::shared_ptr<GraphManager> graph_manager,
                                         std::vector<int> &to_frontier_ids,
                                         std::vector<int> &to_home_ids) {
  if (planning_global_path_pub_.getNumSubscribers() < 1) return;

  std::unordered_map<int, Vertex *> &v_map = graph_manager->vertices_map_;

  visualization_msgs::MarkerArray marker_array;

  // Plot the current to frontier path first.
  visualization_msgs::Marker frontier_marker;
  frontier_marker.header.stamp = ros::Time::now();
  frontier_marker.header.seq = 0;
  frontier_marker.header.frame_id = world_frame_id;
  frontier_marker.id = 0;
  frontier_marker.ns = "current2frontier";
  frontier_marker.action = visualization_msgs::Marker::ADD;
  frontier_marker.type = visualization_msgs::Marker::LINE_LIST;
  frontier_marker.scale.x = 0.4;
  frontier_marker.color.r = 0.0;
  frontier_marker.color.g = 1.0;
  frontier_marker.color.b = 0.0;
  frontier_marker.color.a = 1.0;
  frontier_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  frontier_marker.frame_locked = false;

  if (to_frontier_ids.size() > 1) {
    for (int i = 0; i < (to_frontier_ids.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[to_frontier_ids[i]]->state[0];
      p1.y = v_map[to_frontier_ids[i]]->state[1];
      p1.z = v_map[to_frontier_ids[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[to_frontier_ids[i + 1]]->state[0];
      p2.y = v_map[to_frontier_ids[i + 1]]->state[1];
      p2.z = v_map[to_frontier_ids[i + 1]]->state[2];
      frontier_marker.points.push_back(p1);
      frontier_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(frontier_marker);

  // Plot the current to frontier path first.
  visualization_msgs::Marker home_marker;
  home_marker.header.stamp = ros::Time::now();
  home_marker.header.seq = 0;
  home_marker.header.frame_id = world_frame_id;
  home_marker.id = 0;
  home_marker.ns = "frontier2home";
  home_marker.action = visualization_msgs::Marker::ADD;
  home_marker.type = visualization_msgs::Marker::LINE_LIST;
  home_marker.scale.x = 0.4;
  home_marker.color.r = 0.5;
  home_marker.color.g = 0.0;
  home_marker.color.b = 0.0;
  home_marker.color.a = 1.0;
  home_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  home_marker.frame_locked = false;

  if (to_home_ids.size() > 1) {
    for (int i = 0; i < (to_home_ids.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[to_home_ids[i]]->state[0];
      p1.y = v_map[to_home_ids[i]]->state[1];
      p1.z = v_map[to_home_ids[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[to_home_ids[i + 1]]->state[0];
      p2.y = v_map[to_home_ids[i + 1]]->state[1];
      p2.z = v_map[to_home_ids[i + 1]]->state[2];
      home_marker.points.push_back(p1);
      home_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(home_marker);
  planning_global_path_pub_.publish(marker_array);
}

void Visualization::visualizeRefPath(const std::vector<geometry_msgs::Pose> &path) {
  if (ref_paths_pub_.getNumSubscribers() < 1) return;

  if (path.empty()) return;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "ref_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.25;
  edge_marker.color.r = 244.0 / 255.0;
  edge_marker.color.g = 66.0 / 255.0;
  edge_marker.color.b = 226.0 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    geometry_msgs::Point p2;
    p2.x = path[i + 1].position.x;
    p2.y = path[i + 1].position.y;
    p2.z = path[i + 1].position.z;
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices.
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.25;
  vertex_marker.scale.y = 0.25;
  vertex_marker.scale.z = 0.25;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 100.0 / 255.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  ref_paths_pub_.publish(marker_array);
}

void Visualization::visualizeVolumetricGain(
    Eigen::Vector3d &bound_min, Eigen::Vector3d &bound_max,
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> &voxels,
    double voxel_size) {
  if (volumetric_gain_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot bounding area.
  visualization_msgs::Marker bound_marker;
  bound_marker.header.stamp = ros::Time::now();
  bound_marker.header.seq = 0;
  bound_marker.header.frame_id = world_frame_id;
  bound_marker.id = 0;
  bound_marker.ns = "bound";
  bound_marker.action = visualization_msgs::Marker::ADD;
  bound_marker.type = visualization_msgs::Marker::CUBE;
  bound_marker.pose.position.x = 0.5 * (bound_min[0] + bound_max[0]);
  bound_marker.pose.position.y = 0.5 * (bound_min[1] + bound_max[1]);
  bound_marker.pose.position.z = 0.5 * (bound_min[2] + bound_max[2]);
  bound_marker.scale.x = bound_max[0] - bound_min[0];
  bound_marker.scale.y = bound_max[1] - bound_min[1];
  bound_marker.scale.z = bound_max[2] - bound_min[2];
  tf::Quaternion quat;
  quat.setEuler(0, 0, 0);
  bound_marker.pose.orientation.x = quat.x();
  bound_marker.pose.orientation.y = quat.y();
  bound_marker.pose.orientation.z = quat.z();
  bound_marker.pose.orientation.w = quat.w();
  bound_marker.color.r = 200.0 / 255.0;
  bound_marker.color.g = 100.0 / 255.0;
  bound_marker.color.b = 0.0;
  bound_marker.color.a = 0.25;
  bound_marker.lifetime = ros::Duration(ws_lifetime);
  bound_marker.frame_locked = false;
  marker_array.markers.push_back(bound_marker);

  // Plot unknow voxels.
  visualization_msgs::Marker unknown_voxel_marker;
  unknown_voxel_marker.header.stamp = ros::Time::now();
  unknown_voxel_marker.header.seq = 0;
  unknown_voxel_marker.header.frame_id = world_frame_id;
  unknown_voxel_marker.id = 0;
  unknown_voxel_marker.ns = "unknown_voxels";
  unknown_voxel_marker.action = visualization_msgs::Marker::ADD;
  unknown_voxel_marker.type = visualization_msgs::Marker::CUBE_LIST;
  unknown_voxel_marker.scale.x = voxel_size;
  unknown_voxel_marker.scale.y = voxel_size;
  unknown_voxel_marker.scale.z = voxel_size;
  unknown_voxel_marker.color.r = 1.0;
  unknown_voxel_marker.color.g = 0.0;
  unknown_voxel_marker.color.b = 0.0;
  unknown_voxel_marker.color.a = 0.8;
  unknown_voxel_marker.lifetime = ros::Duration(graph_lifetime);
  unknown_voxel_marker.frame_locked = false;

  visualization_msgs::Marker free_voxel_marker;
  free_voxel_marker.header.stamp = ros::Time::now();
  free_voxel_marker.header.seq = 0;
  free_voxel_marker.header.frame_id = world_frame_id;
  free_voxel_marker.id = 0;
  free_voxel_marker.ns = "free_voxels";
  free_voxel_marker.action = visualization_msgs::Marker::ADD;
  free_voxel_marker.type = visualization_msgs::Marker::CUBE_LIST;
  free_voxel_marker.scale.x = voxel_size;
  free_voxel_marker.scale.y = voxel_size;
  free_voxel_marker.scale.z = voxel_size;
  free_voxel_marker.color.r = 0.0;
  free_voxel_marker.color.g = 1.0;
  free_voxel_marker.color.b = 0.0;
  free_voxel_marker.color.a = 0.8;
  free_voxel_marker.lifetime = ros::Duration(graph_lifetime);
  free_voxel_marker.frame_locked = false;

  visualization_msgs::Marker occupied_voxel_marker;
  occupied_voxel_marker.header.stamp = ros::Time::now();
  occupied_voxel_marker.header.seq = 0;
  occupied_voxel_marker.header.frame_id = world_frame_id;
  occupied_voxel_marker.id = 0;
  occupied_voxel_marker.ns = "occupied_voxels";
  occupied_voxel_marker.action = visualization_msgs::Marker::ADD;
  occupied_voxel_marker.type = visualization_msgs::Marker::CUBE_LIST;
  occupied_voxel_marker.scale.x = voxel_size;
  occupied_voxel_marker.scale.y = voxel_size;
  occupied_voxel_marker.scale.z = voxel_size;
  occupied_voxel_marker.color.r = 0.0;
  occupied_voxel_marker.color.g = 0.0;
  occupied_voxel_marker.color.b = 1.0;
  occupied_voxel_marker.color.a = 0.4;
  occupied_voxel_marker.lifetime = ros::Duration(graph_lifetime);
  occupied_voxel_marker.frame_locked = false;

  int num_voxels = voxels.size();
  for (auto &v : voxels) {
    geometry_msgs::Point p;
    p.x = v.first[0];
    p.y = v.first[1];
    p.z = v.first[2];
    if (v.second == MapManager::VoxelStatus::kUnknown) {
      unknown_voxel_marker.points.push_back(p);
    } else if (v.second == MapManager::VoxelStatus::kFree) {
      free_voxel_marker.points.push_back(p);
    } else if (v.second == MapManager::VoxelStatus::kOccupied) {
      occupied_voxel_marker.points.push_back(p);
    } else {
      ROS_ERROR("Unsupported voxel type.");
    }
  }
  marker_array.markers.push_back(unknown_voxel_marker);
  marker_array.markers.push_back(free_voxel_marker);
  marker_array.markers.push_back(occupied_voxel_marker);

  volumetric_gain_pub_.publish(marker_array);
}

void Visualization::visualizeRays(const StateVec state,
                                  const std::vector<Eigen::Vector3d> ray_endpoints) {
  if (rays_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all rays.
  visualization_msgs::Marker ray_marker;
  ray_marker.header.stamp = ros::Time::now();
  ray_marker.header.seq = 0;
  ray_marker.header.frame_id = world_frame_id;
  ray_marker.id = 0;
  ray_marker.ns = "rays";
  ray_marker.action = visualization_msgs::Marker::ADD;
  ray_marker.type = visualization_msgs::Marker::LINE_LIST;
  ray_marker.scale.x = 0.2;
  ray_marker.color.r = 100.0 / 255.0;
  ray_marker.color.g = 0.0;
  ray_marker.color.b = 0.0;
  ray_marker.color.a = 0.8;
  ray_marker.lifetime = ros::Duration(ray_lifetime);
  ray_marker.frame_locked = false;

  geometry_msgs::Point p0;
  p0.x = state[0];
  p0.y = state[1];
  p0.z = state[2];
  ROS_INFO("Visualize: %d rays", ray_endpoints.size());
  for (auto &ray : ray_endpoints) {
    geometry_msgs::Point p1;
    p1.x = ray[0];
    p1.y = ray[1];
    p1.z = ray[2];
    ray_marker.points.push_back(p0);
    ray_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(ray_marker);
  rays_pub_.publish(marker_array);
}

void Visualization::visualizePCL(const pcl::PointCloud<pcl::PointXYZ> *pcl) {
  if (pcl_pub_.getNumSubscribers() < 1) return;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*pcl, output);
  output.header.frame_id = world_frame_id;
  output.header.stamp = ros::Time::now();
  pcl_pub_.publish(output);
}

void Visualization::visualizeHyperplanes(Eigen::Vector3d &center,
                                         std::vector<Eigen::Vector3d> &hyperplane_list,
                                         std::vector<Eigen::Vector3d> &tangent_point_list) {
  static int markers_id = 0;

  ++markers_id;

  if (hyperplanes_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker proj_points;
  proj_points.header.stamp = ros::Time::now();
  proj_points.header.seq = 0;
  proj_points.header.frame_id = world_frame_id;
  proj_points.id = markers_id;
  proj_points.ns = "planes";
  proj_points.action = visualization_msgs::Marker::ADD;
  proj_points.type = visualization_msgs::Marker::CUBE_LIST;
  proj_points.scale.x = 0.5;
  proj_points.scale.y = 0.5;
  proj_points.scale.z = 0.5;
  proj_points.color.r = 0.0;
  proj_points.color.g = 0.0;
  proj_points.color.b = 1.0;
  proj_points.color.a = 0.8;
  proj_points.lifetime = ros::Duration(graph_lifetime);
  proj_points.frame_locked = false;

  int marker_id = 0;
  for (int i = 0; i < hyperplane_list.size(); ++i) {
    // Get projection point
    // double t0 = -(hyperplane_list[i].dot(center) - 1) /
    // (hyperplane_list[i].squaredNorm());
    // Eigen::Vector3d x_proj = center + hyperplane_list[i] * t0;
    // geometry_msgs::Point p;
    // p.x = x_proj[0];
    // p.y = x_proj[1];
    // p.z = x_proj[2];
    // proj_points.points.push_back(p);

    geometry_msgs::Point p;
    p.x = tangent_point_list[i][0];
    p.y = tangent_point_list[i][1];
    p.z = tangent_point_list[i][2];
    proj_points.points.push_back(p);

    Eigen::Quaternion<double> quat_W2S;
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    quat_W2S.setFromTwoVectors(x_axis, hyperplane_list[i].normalized());
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = world_frame_id;
    marker.ns = "normal";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 1.0;   // length of the arrow
    marker.scale.y = 0.15;  // arrow width
    marker.scale.z = 0.15;  // arrow height
    marker.color.r = 200.0 / 255.0;
    marker.color.g = 50.0 / 255.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(graph_lifetime);
    marker.frame_locked = false;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;
    marker.pose.orientation.x = quat_W2S.x();
    marker.pose.orientation.y = quat_W2S.y();
    marker.pose.orientation.z = quat_W2S.z();
    marker.pose.orientation.w = quat_W2S.w();
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker marker_plane;
    marker_plane.header.stamp = ros::Time::now();
    marker_plane.header.seq = 0;
    marker_plane.header.frame_id = world_frame_id;
    marker_plane.ns = "plane";
    marker_plane.action = visualization_msgs::Marker::ADD;
    marker_plane.type = visualization_msgs::Marker::CUBE;
    marker_plane.scale.x = 0.1;  // length of the arrow
    marker_plane.scale.y = 5.0;  // arrow width
    marker_plane.scale.z = 5.0;  // arrow height
    marker_plane.color.r = 200.0 / 255.0;
    marker_plane.color.g = 50.0 / 255.0;
    marker_plane.color.b = 0.0;
    marker_plane.color.a = 0.5;
    marker_plane.lifetime = ros::Duration(graph_lifetime);
    marker_plane.frame_locked = false;
    marker_plane.pose.position.x = p.x;
    marker_plane.pose.position.y = p.y;
    marker_plane.pose.position.z = p.z;
    marker_plane.pose.orientation.x = quat_W2S.x();
    marker_plane.pose.orientation.y = quat_W2S.y();
    marker_plane.pose.orientation.z = quat_W2S.z();
    marker_plane.pose.orientation.w = quat_W2S.w();
    marker_plane.id = marker_id++;
    marker_array.markers.push_back(marker_plane);
  }
  marker_array.markers.push_back(proj_points);

  visualization_msgs::Marker tangent_points;
  tangent_points.header.stamp = ros::Time::now();
  tangent_points.header.seq = 0;
  tangent_points.header.frame_id = world_frame_id;
  tangent_points.id = 0;
  tangent_points.ns = "points";
  tangent_points.action = visualization_msgs::Marker::ADD;
  tangent_points.type = visualization_msgs::Marker::CUBE_LIST;
  tangent_points.scale.x = 0.5;
  tangent_points.scale.y = 0.5;
  tangent_points.scale.z = 0.5;
  tangent_points.color.r = 1.0;
  tangent_points.color.g = 0.0;
  tangent_points.color.b = 0.0;
  tangent_points.color.a = 0.8;
  tangent_points.lifetime = ros::Duration(graph_lifetime);
  tangent_points.frame_locked = false;

  for (int i = 0; i < tangent_point_list.size(); ++i) {
    geometry_msgs::Point p;
    p.x = tangent_point_list[i][0];
    p.y = tangent_point_list[i][1];
    p.z = tangent_point_list[i][2];
    tangent_points.points.push_back(p);
  }

  marker_array.markers.push_back(tangent_points);

  hyperplanes_pub_.publish(marker_array);
}

void Visualization::visualizeModPath(const std::vector<geometry_msgs::Pose> &path) {
  if (mod_path_pub_.getNumSubscribers() < 1) return;

  if (path.empty()) return;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "mod_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.25;
  edge_marker.color.r = 244.0 / 255.0;
  edge_marker.color.g = 217 / 255.0;
  edge_marker.color.b = 66 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    geometry_msgs::Point p2;
    p2.x = path[i + 1].position.x;
    p2.y = path[i + 1].position.y;
    p2.z = path[i + 1].position.z;
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);
  mod_path_pub_.publish(marker_array);
}

void Visualization::visualizeMBPath(const std::vector<geometry_msgs::Pose> &final_path,
                                    const std::vector<geometry_msgs::Pose> &full_path,
                                    const std::vector<geometry_msgs::Pose> &original_path) {
  nav_msgs::Path viz_final_path, viz_full_path, viz_original_path;
  viz_final_path.header.frame_id = world_frame_id;
  viz_full_path.header.frame_id = world_frame_id;
  viz_original_path.header.frame_id = world_frame_id;

  for (int i = 0; i < final_path.size(); i++) {
    geometry_msgs::Pose p = final_path[i];
    geometry_msgs::PoseStamped ps;
    ps.pose = p;
    viz_final_path.poses.push_back(ps);
  }
  for (int i = 0; i < full_path.size(); i++) {
    geometry_msgs::Pose p = full_path[i];
    geometry_msgs::PoseStamped ps;
    ps.pose = p;
    viz_full_path.poses.push_back(ps);
  }
  for (int i = 0; i < original_path.size(); i++) {
    geometry_msgs::Pose p = original_path[i];
    geometry_msgs::PoseStamped ps;
    ps.pose = p;
    viz_original_path.poses.push_back(ps);
  }

  viz_final_path_pub_.publish(viz_final_path);
  viz_full_path_pub_.publish(viz_full_path);
  viz_original_path_pub_.publish(viz_original_path);
}

void Visualization::visualizeSensorFOV(StateVec &state, SensorParams &sensor_params) {
  if (sensor_fov_pub_.getNumSubscribers() < 1) return;
  if (sensor_params.sensor_list.size() == 0) return;

  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < sensor_params.sensor_list.size(); ++i) {
    std::string sensor_name = sensor_params.sensor_list[i];
    SensorParamsBase *sb = &(sensor_params.sensor[sensor_name]);
    if (sb->type == SensorType::kCamera) {
      // Visualize its frustum using lines precomputed.
      visualization_msgs::Marker line_marker;
      line_marker.header.stamp = ros::Time::now();
      line_marker.header.seq = 0;
      line_marker.header.frame_id = world_frame_id;
      line_marker.ns = sensor_name;
      line_marker.id = 0;
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.type = visualization_msgs::Marker::LINE_LIST;
      line_marker.scale.x = 0.1;
      line_marker.color.r = 0.0;
      line_marker.color.g = 100.0 / 255.0;
      line_marker.color.b = 1.0;
      line_marker.color.a = 1.0;
      line_marker.lifetime = ros::Duration(robot_lifetime);
      line_marker.frame_locked = false;

      // Rotate the edges from body to world coordinate.
      std::vector<Eigen::Vector3d> edge_points_w;
      sb->getFrustumEdges(state, edge_points_w);
      // Add lines
      for (int j = 0; j < 4; ++j) {
        geometry_msgs::Point p1;
        p1.x = state[0];
        p1.y = state[1];
        p1.z = state[2];
        geometry_msgs::Point p2;
        p2.x = edge_points_w[j][0];
        p2.y = edge_points_w[j][1];
        p2.z = edge_points_w[j][2];
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
      }
      for (int j = 0; j < 3; ++j) {
        geometry_msgs::Point p1;
        p1.x = edge_points_w[j][0];
        p1.y = edge_points_w[j][1];
        p1.z = edge_points_w[j][2];
        geometry_msgs::Point p2;
        p2.x = edge_points_w[j + 1][0];
        p2.y = edge_points_w[j + 1][1];
        p2.z = edge_points_w[j + 1][2];
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
      }
      {
        geometry_msgs::Point p1;
        p1.x = edge_points_w[3][0];
        p1.y = edge_points_w[3][1];
        p1.z = edge_points_w[3][2];
        geometry_msgs::Point p2;
        p2.x = edge_points_w[0][0];
        p2.y = edge_points_w[0][1];
        p2.z = edge_points_w[0][2];
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
      }
      marker_array.markers.push_back(line_marker);
    } else if (sb->type == SensorType::kLidar) {
      // Visualize as a cylinder (not correct but for debug purpose).
      visualization_msgs::Marker cyl_marker;
      cyl_marker.header.stamp = ros::Time::now();
      cyl_marker.header.seq = 0;
      cyl_marker.header.frame_id = world_frame_id;
      cyl_marker.ns = sensor_name;
      cyl_marker.action = visualization_msgs::Marker::ADD;
      cyl_marker.type = visualization_msgs::Marker::CYLINDER;
      cyl_marker.scale.x = sb->max_range * 2;                        // x-diameter
      cyl_marker.scale.y = sb->max_range * 2;                        // y-diameter
      cyl_marker.scale.z = sb->max_range * sin(sb->fov[1] / 2) * 2;  // height
      cyl_marker.pose.position.x = state[0];
      cyl_marker.pose.position.y = state[1];
      cyl_marker.pose.position.z = state[2];
      Eigen::Matrix3d rot_W2B;
      rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      Eigen::Matrix3d rot_B2S;  // body to sensor from yaml setting.
      rot_B2S = Eigen::AngleAxisd(sb->rotations[0], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(sb->rotations[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(sb->rotations[2], Eigen::Vector3d::UnitX());
      Eigen::Matrix3d rot_W2S = rot_W2B * rot_B2S;  // Convert point from sensor to body
      Eigen::Quaterniond quat(rot_W2S);
      cyl_marker.pose.orientation.x = quat.x();
      cyl_marker.pose.orientation.y = quat.y();
      cyl_marker.pose.orientation.z = quat.z();
      cyl_marker.pose.orientation.w = quat.w();
      cyl_marker.color.r = 0.0;
      cyl_marker.color.g = 100.0 / 255.0;
      cyl_marker.color.b = 1.0;
      cyl_marker.color.a = 0.5;
      cyl_marker.lifetime = ros::Duration(robot_lifetime);
      cyl_marker.frame_locked = false;
      marker_array.markers.push_back(cyl_marker);
    }
  }
  sensor_fov_pub_.publish(marker_array);
}

void Visualization::visualizeRobotState(const StateNode &state, const Eigen::Vector3d size,
                                        const double &direction) {
  // Robot state box
  visualization_msgs::Marker robot_box;
  robot_box.header.stamp = ros::Time::now();
  robot_box.header.seq = 0;
  robot_box.header.frame_id = world_frame_id;
  robot_box.id = 0;
  robot_box.ns = "robot_box";
  robot_box.action = visualization_msgs::Marker::ADD;
  robot_box.type = visualization_msgs::Marker::CUBE;
  robot_box.scale.x = size(0);
  robot_box.scale.y = size(1);
  robot_box.scale.z = size(2);
  robot_box.color.r = 200.0 / 255.0;
  robot_box.color.g = 200.0 / 255.0;
  robot_box.color.b = 200.0 / 255.0;
  robot_box.color.a = 0.7;
  robot_box.lifetime = ros::Duration(0.0);
  robot_box.frame_locked = false;
  robot_box.pose.position.x = state.position.x();
  robot_box.pose.position.y = state.position.y();
  robot_box.pose.position.z = state.position.z();
  tf::Quaternion quat_yaw;
  quat_yaw.setEuler(0.0, 0.0, state.yaw);
  robot_box.pose.orientation.x = quat_yaw[0];
  robot_box.pose.orientation.y = quat_yaw[1];
  robot_box.pose.orientation.z = quat_yaw[2];
  robot_box.pose.orientation.w = quat_yaw[3];
  robot_box_pub_.publish(robot_box);

  // Exploration direction
  geometry_msgs::PoseStamped exp_dir_pose;
  exp_dir_pose.header.frame_id = world_frame_id;
  exp_dir_pose.pose.position.x = state.position(0);
  exp_dir_pose.pose.position.y = state.position(1);
  exp_dir_pose.pose.position.z = state.position(2);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, direction);
  exp_dir_pose.pose.orientation.x = quat[0];
  exp_dir_pose.pose.orientation.y = quat[1];
  exp_dir_pose.pose.orientation.z = quat[2];
  exp_dir_pose.pose.orientation.w = quat[3];
  exploration_direction_pub_.publish(exp_dir_pose);
}

void Visualization::visualizeSimilarPaths(
    const std::vector<std::vector<geometry_msgs::Pose>> &valid_paths,
    const std::vector<std::vector<geometry_msgs::Pose>> &invalid_paths) {
  visualization_msgs::MarkerArray similar_paths_marker_array;

  ROS_INFO("Valid paths: %d, invalid paths: %d", valid_paths.size(), invalid_paths.size());

  visualization_msgs::Marker valid_paths_marker;
  valid_paths_marker.header.stamp = ros::Time::now();
  valid_paths_marker.header.seq = 0;
  valid_paths_marker.header.frame_id = world_frame_id;
  valid_paths_marker.id = 0;
  valid_paths_marker.ns = "valid_similar_paths";
  valid_paths_marker.action = visualization_msgs::Marker::ADD;
  valid_paths_marker.type = visualization_msgs::Marker::LINE_LIST;
  valid_paths_marker.scale.x = 0.09;
  valid_paths_marker.color.r = 00.0 / 255.0;
  valid_paths_marker.color.g = 0.0 / 255.0;
  valid_paths_marker.color.b = 255.0 / 255.0;
  valid_paths_marker.color.a = 0.6;
  valid_paths_marker.lifetime = ros::Duration(0.0);
  valid_paths_marker.frame_locked = false;

  if (valid_paths.size() > 0) {
    for (int i = 0; i < valid_paths.size(); i++) {
      std::vector<geometry_msgs::Pose> curr_path = valid_paths[i];
      for (int j = 0; j < curr_path.size() - 1; j++) {
        geometry_msgs::Point p1;
        p1 = curr_path[j].position;
        geometry_msgs::Point p2;
        p2 = curr_path[j + 1].position;
        valid_paths_marker.points.push_back(p1);
        valid_paths_marker.points.push_back(p2);
      }
    }
    similar_paths_marker_array.markers.push_back(valid_paths_marker);
  }

  visualization_msgs::Marker invalid_paths_marker;
  invalid_paths_marker.header.stamp = ros::Time::now();
  invalid_paths_marker.header.seq = 0;
  invalid_paths_marker.header.frame_id = world_frame_id;
  invalid_paths_marker.id = 0;
  invalid_paths_marker.ns = "invalid_similar_paths";
  invalid_paths_marker.action = visualization_msgs::Marker::ADD;
  invalid_paths_marker.type = visualization_msgs::Marker::LINE_LIST;
  invalid_paths_marker.scale.x = 0.09;
  invalid_paths_marker.color.r = 00.0 / 255.0;
  invalid_paths_marker.color.g = 0.0 / 255.0;
  invalid_paths_marker.color.b = 100.0 / 255.0;
  invalid_paths_marker.color.a = 0.6;
  invalid_paths_marker.lifetime = ros::Duration(0.0);
  invalid_paths_marker.frame_locked = false;

  if (invalid_paths.size() > 0) {
    for (int i = 0; i < invalid_paths.size(); i++) {
      std::vector<geometry_msgs::Pose> curr_path = invalid_paths[i];
      for (int j = 0; j < curr_path.size() - 1; j++) {
        geometry_msgs::Point p1;
        p1 = curr_path[j].position;
        geometry_msgs::Point p2;
        p2 = curr_path[j + 1].position;
        invalid_paths_marker.points.push_back(p1);
        invalid_paths_marker.points.push_back(p2);
      }
    }
    similar_paths_marker_array.markers.push_back(invalid_paths_marker);
  }

  if (valid_paths.size() > 0 || invalid_paths.size() > 0)
    similar_paths_pub_.publish(similar_paths_marker_array);
  ROS_INFO("Similar paths published");
}

void Visualization::visualizeMBTree(StateNode *root) {
  visualization_msgs::MarkerArray tree_marker;

  visualization_msgs::Marker vertices, edges;
  vertices.header.stamp = ros::Time::now();
  vertices.header.seq = 0;
  vertices.header.frame_id = world_frame_id;
  vertices.id = 0;
  vertices.ns = "vertices";
  vertices.action = visualization_msgs::Marker::ADD;
  vertices.type = visualization_msgs::Marker::SPHERE_LIST;
  vertices.scale.x = 0.15;
  vertices.scale.y = 0.15;
  vertices.scale.z = 0.15;
  vertices.color.r = 53.0 / 255.0;
  vertices.color.g = 49.0 / 255.0;
  vertices.color.b = 119.0 / 255.0;
  vertices.color.a = 1.0;
  vertices.lifetime = ros::Duration(0.0);
  vertices.frame_locked = false;

  edges.header.stamp = ros::Time::now();
  edges.header.seq = 0;
  edges.header.frame_id = world_frame_id;
  edges.id = 0;
  edges.ns = "edges";
  edges.action = visualization_msgs::Marker::ADD;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.scale.x = 0.015;
  edges.color.r = 255.0 / 255.0;
  edges.color.g = 100.0 / 255.0;
  edges.color.b = 000.0 / 255.0;
  edges.color.a = 1.0;
  edges.lifetime = ros::Duration(0.0);
  edges.frame_locked = false;

  std::queue<StateNode *> q;  // Create a queue
  q.push(root);
  while (!q.empty()) {
    geometry_msgs::Point p1;
    p1.x = q.front()->position.x();
    p1.y = q.front()->position.y();
    p1.z = q.front()->position.z();
    vertices.points.push_back(p1);
    if (q.front()->children.size() != 0) {
      for (int i = 0; i < q.front()->children.size(); i++) {
        geometry_msgs::Point p2;
        p2.x = q.front()->children[i]->position.x();
        p2.y = q.front()->children[i]->position.y();
        p2.z = q.front()->children[i]->position.z();
        edges.points.push_back(p1);
        edges.points.push_back(p2);

        q.push(q.front()->children[i]);
      }
    }
    q.pop();
  }
  tree_marker.markers.push_back(vertices);
  tree_marker.markers.push_back(edges);
  viz_tree_pub_.publish(tree_marker);
}

void Visualization::visualizeMBTreeEvaluation(std::vector<StateNode *> &leaf_vertices) {
  visualization_msgs::MarkerArray gain_texts;
  visualization_msgs::Marker safety_paths;

  safety_paths.header.stamp = ros::Time::now();
  safety_paths.header.seq = 0;
  safety_paths.header.frame_id = world_frame_id;
  safety_paths.id = 0;
  safety_paths.ns = "safety_paths";
  safety_paths.action = visualization_msgs::Marker::ADD;
  safety_paths.type = visualization_msgs::Marker::LINE_LIST;
  safety_paths.scale.x = 0.05;
  safety_paths.color.r = 255.0 / 255.0;
  safety_paths.color.g = 000.0 / 255.0;
  safety_paths.color.b = 000.0 / 255.0;
  safety_paths.color.a = 1.0;
  safety_paths.lifetime = ros::Duration(0.0);
  safety_paths.frame_locked = false;

  int text_id = 0;
  for (int i = 0; i < leaf_vertices.size(); i++) {
    if (leaf_vertices[i]->safe_path.size() > 0) {
      geometry_msgs::Point p2;
      geometry_msgs::Point p1;
      p1.x = leaf_vertices[i]->safe_path[0]->position.x();
      p1.y = leaf_vertices[i]->safe_path[0]->position.y();
      p1.z = leaf_vertices[i]->safe_path[0]->position.z();
      p2.x = leaf_vertices[i]->safe_path.back()->position.x();
      p2.y = leaf_vertices[i]->safe_path.back()->position.y();
      p2.z = leaf_vertices[i]->safe_path.back()->position.z();
      safety_paths.points.push_back(p1);
      safety_paths.points.push_back(p2);
    }

    visualization_msgs::Marker gain_text;
    gain_text.header.stamp = ros::Time::now();
    gain_text.header.seq = 0;
    gain_text.header.frame_id = world_frame_id;
    gain_text.id = text_id++;
    gain_text.ns = "gain";
    gain_text.action = visualization_msgs::Marker::ADD;
    gain_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    gain_text.scale.z = 0.05;
    gain_text.color.r = 1.0;
    gain_text.color.g = 1.0;
    gain_text.color.b = 1.0;
    gain_text.color.a = 1.0;
    gain_text.lifetime = ros::Duration(0.0);
    gain_text.frame_locked = false;
    gain_text.text = std::to_string(leaf_vertices[i]->vol_gain.num_unknown_voxels) + " | " +
                     std::to_string(leaf_vertices[i]->final_gain);
    gain_text.pose.position.x = leaf_vertices[i]->position(0);
    gain_text.pose.position.y = leaf_vertices[i]->position(1);
    gain_text.pose.position.z = leaf_vertices[i]->position(2);
    gain_text.pose.orientation.x = 0.0;
    gain_text.pose.orientation.y = 0.0;
    gain_text.pose.orientation.z = 0.0;
    gain_text.pose.orientation.w = 1.0;
    gain_texts.markers.push_back(gain_text);
  }

  safety_paths_pub_.publish(safety_paths);
  viz_gains_pub_.publish(gain_texts);
  ROS_INFO("Tree eval visualized");
}
}