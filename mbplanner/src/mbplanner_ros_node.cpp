/* Author: Mihir Dharmadhikari */
/* This is where the code starts. */
#include "mbplanner/mbplanner.h"
#include <ros/ros.h>

class MyVector3 {
 public:
  double x;
  double y;
  double z;
  void print() { std::cout << x << " " << y << " " << z << std::endl; }
};
class TwoVectors {
 public:
  MyVector3 v1;
  MyVector3 v2;
  void print() {
    v1.print();
    v2.print();
  }
};
class VectorArray {
 public:
  std::vector<MyVector3> v;
  void print() {
    for (int i = 0; i < v.size(); i++) v[i].print();
  }
};
enum class VertexType {
  kUnvisited = 0,  // Detect as a collision free vertex to visit but hasn't done yet.
  kVisited = 1,    // Free and visisted.
  kFrontier = 2    // Potential frontier vertex to explore, hasn't visited yet.
};
// compile-time assert that sizeof(MyVector3) == serializationLength(x) +
// serializationLength(y) + serializationLength(z)
ROS_STATIC_ASSERT(sizeof(MyVector3) == 24);
ROS_STATIC_ASSERT(sizeof(TwoVectors) == 48);
ROS_STATIC_ASSERT(sizeof(VertexType) == 4);
// ROS_STATIC_ASSERT(sizeof(VectorArray) == 24);

namespace ros {

namespace serialization {
template <>
struct Serializer<MyVector3> {
  template <typename Stream>
  inline static void write(Stream& stream, const MyVector3& t) {
    stream.next(t.x);
    stream.next(t.y);
    stream.next(t.z);
  }

  template <typename Stream>
  inline static void read(Stream& stream, MyVector3& t) {
    stream.next(t.x);
    stream.next(t.y);
    stream.next(t.z);
  }

  inline static uint32_t serializedLength(const MyVector3& t) {
    uint32_t size = 0;
    size += serializationLength(t.x);
    size += serializationLength(t.y);
    size += serializationLength(t.z);
    return size;
  }
};

template <>
struct Serializer<TwoVectors> {
  template <typename Stream>
  inline static void write(Stream& stream, const TwoVectors& t) {
    stream.next(t.v1);
    stream.next(t.v2);
  }

  template <typename Stream>
  inline static void read(Stream& stream, TwoVectors& t) {
    stream.next(t.v1);
    stream.next(t.v2);
  }

  inline static uint32_t serializedLength(const TwoVectors& t) {
    uint32_t size = 0;
    size += serializationLength(t.v1);
    size += serializationLength(t.v2);
    return size;
  }
};

template <>
struct Serializer<VectorArray> {
  template <typename Stream>
  inline static void write(Stream& stream, const VectorArray& t) {
    std::cout << "SIZE write: " << t.v.size() << std::endl;
    for (int i = 0; i < t.v.size(); i++) stream.next(t.v[i]);
  }

  template <typename Stream>
  inline static void read(Stream& stream, VectorArray& t) {
    std::cout << "SIZE read: " << t.v.size() << std::endl;
    for (int i = 0; i < t.v.size(); i++) stream.next(t.v[i]);
  }

  inline static uint32_t serializedLength(const VectorArray& t) {
    uint32_t size = 0;
    for (int i = 0; i < t.v.size(); i++) size += serializationLength(t.v[i]);
    std::cout << "SIZE: " << size << std::endl;
    return size;
  }
};

template <>
struct Serializer<VertexType> {
  template <typename Stream>
  inline static void write(Stream& stream, const VertexType& t) {
    stream.next(t);
  }

  template <typename Stream>
  inline static void read(Stream& stream, VertexType& t) {
    stream.next(t);
  }

  inline static uint32_t serializedLength(const VertexType& t) {
    uint32_t size = 0;
    size += serializationLength(t);
    std::cout << "SIZE: " << size << std::endl;
    return size;
  }
};

}  // namespace serialization
}  // namespace ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "mbplanner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  explorer::MBPlanner mbtree(nh, nh_private);

  ros::spin();

  return 0;
}
