#include <ros/ros.h>
#include "global_planner/global_planner_manager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  explorer::GPManager global_planner(nh, nh_private);

  ros::spin();

  return 0;
}
