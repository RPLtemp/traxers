#include <traxers_nav/occ_grid_to_costmap_node.h>

namespace traxers {

OccGridToCostmapNode::OccGridToCostmapNode() :
    received_map_(false) {
  // Create a private ros node for reading parameters
  ros::NodeHandle pnh("~");

  // Retrieve all parameters or set to default
  pnh.param("start_x", start_pt_.x, kDefaultStartX);
  pnh.param("start_y", start_pt_.y, kDefaultStartY);
  pnh.param("is_circular", is_circular_, kDefaultIsCircular);

  if (!is_circular_) {
    pnh.param("goal_x", goal_pt_.x, kDefaultStartX);
    pnh.param("goal_y", goal_pt_.y, kDefaultStartY);

    if (start_pt_ == goal_pt_) {
      is_circular_ = true;
      ROS_INFO("Start point and goal point are the same, assuming circular"
               " desired path");
    }
  }

  std::string grid_pub_topic;
  std::string map_sub_topic;

  pnh.param("grid_pub_topic", grid_pub_topic, kDefaultGridPubTopic);
  pnh.param("map_sub_topic", map_sub_topic, kDefaultMapSubTopic);

  // Advertise the occupancy grid publisher
  grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(grid_pub_topic, 1, true);

  // Subscribe to the map message
  map_sub_ = nh_.subscribe(map_sub_topic, 1,
                           &OccGridToCostmapNode::MapCallback, this);
}

OccGridToCostmapNode::~OccGridToCostmapNode() {
}

void OccGridToCostmapNode::CreateCostmap() {
  // The occupancy grid utils object should be initialized by now
  assert(occ_grid_utils_);
}

void OccGridToCostmapNode::PublishGrid() {
  grid_pub_.publish(occ_grid_utils_->GetGrid());
  ros::spinOnce();
}

void OccGridToCostmapNode::MapCallback(
    const nav_msgs::OccupancyGridConstPtr& map_msg) {
  ROS_INFO("Received map");

  // Initialize the occupancy grid utils object
  occ_grid_utils_ = boost::make_shared<OccGridUtils>(map_msg);

  received_map_ = true;
  map_sub_.shutdown();
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "occ_grid_to_costmap_node");

  traxers::OccGridToCostmapNode occ_grid_to_costmap_node;

  // Wait until we receive a map.
  ros::Rate r(10);
  while (!occ_grid_to_costmap_node.ReceivedMap() && ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  // Create the costmap
  occ_grid_to_costmap_node.CreateCostmap();

  return 0;
}
