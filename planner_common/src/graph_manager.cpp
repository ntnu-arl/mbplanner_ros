#include "planner_common/graph_manager.h"
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include "ros/ros.h"

namespace explorer {

// ROS_STATIC_ASSERT(sizeof(MyVector3) == 24);
// ROS_STATIC_ASSERT(sizeof(VolumetricGain) == 28);

GraphManager::GraphManager() {
  kd_tree_ = NULL;
  reset();
}

void GraphManager::reset() {
  // Reset kdtree first.
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);

  // Reset graph.
  graph_.reset(new Graph());

  // Vertex mapping.
  vertices_map_.clear();

  // Other params.
  subgraph_ind_ = -1;
  id_count_ = -1;
}

void GraphManager::addVertex(Vertex* v) {
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
  if (v->id == 0)
    graph_->addSourceVertex(0);
  else
    graph_->addVertex(v->id);
  vertices_map_[v->id] = v;
}

void GraphManager::addEdge(Vertex* v, Vertex* u, double weight) {
  graph_->addEdge(v->id, u->id, weight);
}

void GraphManager::removeEdge(Vertex* v, Vertex* u) { graph_->removeEdge(v->id, u->id); }

bool GraphManager::getNearestVertex(const StateVec* state, Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool GraphManager::getNearestVertexInRange(const StateVec* state, double range,
                                           Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*v_res)->state.x(), state->y() - (*v_res)->state.y(),
      state->z() - (*v_res)->state.z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}

bool GraphManager::getNearestVertices(const StateVec* state, double range,
                                      std::vector<Vertex*>* v_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  kdres* neighbors = kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  v_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    Vertex* new_neighbor = (Vertex*)kd_res_item_data(neighbors);
    v_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool GraphManager::findShortestPaths(ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(0, rep);
}

bool GraphManager::findShortestPaths(int source_id, ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(source_id, rep);
}

void GraphManager::getShortestPath(int target_id, const ShortestPathsReport& rep,
                                   bool source_to_target_order, std::vector<Vertex*>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.push_back(vertices_map_[*p]);
  }
}

void GraphManager::getShortestPath(int target_id, const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Eigen::Vector3d>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(Eigen::Vector3d(vertices_map_[*p]->state.x(),
                                      vertices_map_[*p]->state.y(),
                                      vertices_map_[*p]->state.z()));
  }
}

void GraphManager::getShortestPath(int target_id, const ShortestPathsReport& rep,
                                   bool source_to_target_order, std::vector<StateVec>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(vertices_map_[*p]->state);
  }
}

void GraphManager::getShortestPath(int target_id, const ShortestPathsReport& rep,
                                   bool source_to_target_order, std::vector<int>& path) {
  path.clear();
  if (!rep.status) {
    ROS_WARN("Shortest paths report is not valid");
    return;
  }

  if (rep.parent_id_map.size() <= target_id) {
    ROS_WARN("Vertext with ID [%d] doesn't exist in the graph", target_id);
    return;
  }

  if (target_id == rep.source_id) {
    path.push_back(target_id);
    return;
  }

  int parent_id = rep.parent_id_map.at(target_id);
  if (parent_id == target_id) {
    ROS_WARN("Vertex with ID [%d] is isolated from the graph", target_id);
    return;
  }

  path.push_back(target_id);  // current vertex id first
  path.push_back(parent_id);  // its first parent, the rest is recursively looked up
  while (parent_id != rep.source_id) {
    parent_id = rep.parent_id_map.at(parent_id);
    path.push_back(parent_id);
  }

  // Initially, the path follows target to source order. Reverse if required.
  if (source_to_target_order) {
    std::reverse(path.begin(), path.end());
  }
}

double GraphManager::getShortestDistance(int target_id, const ShortestPathsReport& rep) {
  double dist = std::numeric_limits<double>::max();

  if (!rep.status) {
    ROS_WARN("Shortest paths report is not valid");
    return dist;
  }
  if (rep.parent_id_map.size() <= target_id) {
    ROS_WARN("Vertex with ID [%d] doesn't exist in the graph", target_id);
    return dist;
  }
  dist = rep.distance_map.at(target_id);
  return dist;
}

int GraphManager::getParentIDFromShortestPath(int target_id, const ShortestPathsReport& rep) {
  if (!rep.status) {
    ROS_WARN("Shortest paths report is not valid");
    return target_id;
  }

  if (rep.parent_id_map.size() <= target_id) {
    ROS_WARN("Vertex with ID [%d] doesn't exist in the graph", target_id);
    return target_id;
  }

  return rep.parent_id_map.at(target_id);
}

void GraphManager::getLeafVertices(std::vector<Vertex*>& leaf_vertices) {
  leaf_vertices.clear();
  for (int id = 0; id < getNumVertices(); ++id) {
    Vertex* v = getVertex(id);
    if (v->is_leaf_vertex) leaf_vertices.push_back(v);
  }
}

void GraphManager::findLeafVertices(const ShortestPathsReport& rep) {
  int num_vertices = getNumVertices();
  for (int id = 0; id < num_vertices; ++id) {
    int pid = getParentIDFromShortestPath(id, rep);
    getVertex(pid)->is_leaf_vertex = false;
  }
}

int GraphManager::generateSubgraphIndex() { return ++subgraph_ind_; }

int GraphManager::generateVertexID() { return ++id_count_; }

void GraphManager::updateVertexTypeInRange(StateVec& state, double range) {
  std::vector<Vertex*> nearest_vertices;
  getNearestVertices(&state, range, &nearest_vertices);
  for (auto& v : nearest_vertices) {
    v->type = VertexType::kVisited;
  }
}

// namespace ros
// {
// namespace serialization
// {

// // template<>
// // struct Serializer<VolumetricGain>
// // {
// // // double gain;
// // // double accumulative_gain;
// // // int num_unknown_voxels;
// // // int num_occupied_voxels;
// // // int num_free_voxels;
// // // bool is_frontier;

// //   template<typename Stream>
// //   inline static void write(Stream& stream, const VolumetricGain& t)
// //   {
// //     stream.next(t.gain);
// //     stream.next(t.accumulative_gain);
// //     stream.next(t.num_unknown_voxels);
// //     stream.next(t.num_occupied_voxels);
// //     stream.next(t.num_free_voxels);
// //     stream.next(t.is_frontier);
// //   }

// //   template<typename Stream>
// //   inline static void read(Stream& stream, VolumetricGain& t)
// //   {
// //     stream.next(t.gain);
// //     stream.next(t.accumulative_gain);
// //     stream.next(t.num_unknown_voxels);
// //     stream.next(t.num_occupied_voxels);
// //     stream.next(t.num_free_voxels);
// //     stream.next(t.is_frontier);
// //   }

// //   inline static uint32_t serializedLength(const VolumetricGain& t)
// //   {
// //     uint32_t size = 0;
// //     size += serializationLength(t.gain);
// //     size += serializationLength(t.accumulative_gain);
// //     size += serializationLength(t.num_unknown_voxels);
// //     size += serializationLength(t.num_occupied_voxels);
// //     size += serializationLength(t.num_free_voxels);
// //     size += serializationLength(t.is_frontier);
// //     return size;
// //   }
// // };

// template<>
// struct Serializer<MyVector3>
// {
//   template<typename Stream>
//   inline static void write(Stream& stream, const MyVector3& t)
//   {
//     stream.next(t.x);
//     stream.next(t.y);
//     stream.next(t.z);
//   }

//   template<typename Stream>
//   inline static void read(Stream& stream, MyVector3& t)
//   {
//     stream.next(t.x);
//     stream.next(t.y);
//     stream.next(t.z);
//   }

//   inline static uint32_t serializedLength(const MyVector3& t)
//   {
//     uint32_t size = 0;
//     size += serializationLength(t.x);
//     size += serializationLength(t.y);
//     size += serializationLength(t.z);
//     return size;
//   }
// };

// } // ser
// } // ros

}  // namespace explorer
