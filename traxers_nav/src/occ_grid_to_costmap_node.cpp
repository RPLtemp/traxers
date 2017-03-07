#include <traxers_nav/occ_grid_to_costmap_node.h>

namespace traxers {

OccGridToCostmapNode::OccGridToCostmapNode() {
  map_sub_ = nh_.subscribe(kMapSubTopic, 1,
                           &OccGridToCostmapNode::MapCallback, this);
}

OccGridToCostmapNode::~OccGridToCostmapNode() {
}

void OccGridToCostmapNode::MapCallback(
    const nav_msgs::OccupancyGridConstPtr& map_msg) {
  ROS_INFO("Received map");

  map_sub_.shutdown();
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "occ_grid_to_costmap_node");

  traxers::OccGridToCostmapNode occ_grid_to_costmap_node;

  ros::spin();

  return 0;
}
