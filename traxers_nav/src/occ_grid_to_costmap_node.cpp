#include "traxers_nav/occ_grid_to_costmap_node.h"

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

  pnh.param("action_extent", action_extent_, kDefaultActionExtent);

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

  // If the desired path is circular, the starting point needs to be blocked
  // off from the goal
  if (is_circular_) {
    // First, move the goal back
    if (!occ_grid_utils_->MoveCells(start_pt_, goal_pt_, -2, 0))
      throw ros::Exception("Goal point coordinate not valid after move");

    // Next, draw a line of occupied cells between start and goal points
    Point wall_point;
    occ_grid_utils_->MoveCells(start_pt_, wall_point, -1, 0);
    occ_grid_utils_->DrawLine(wall_point, 0, 1);
  }

  // Initialize the value iteration object with a state space created from our
  // occupancy grid map
  value_iteration_.Init(OccGridToStateSpace(occ_grid_utils_->GetGrid()));

  // Add the actions
  std::vector<int> action_extents = {-action_extent_, action_extent_};
  for (int x : action_extents) {
    for (int y = -action_extent_; y <= action_extent_; y++) {
      value_iteration_.AddAction(Action(x, y));
    }
  }

  for (int y : action_extents) {
    for (int x = -(action_extent_ - 1); x < action_extent_; x++) {
      value_iteration_.AddAction(Action(x, y));
    }
  }

  // Run the value iteration
  value_iteration_.Run();
}

StateSpace2D OccGridToCostmapNode::OccGridToStateSpace(
    const nav_msgs::OccupancyGrid& occ_grid) {
  StateSpace2D state_space(occ_grid.data,
                           occ_grid.info.width,
                           occ_grid.info.height);

  return state_space;
}

void OccGridToCostmapNode::PublishGrid() {
  grid_pub_.publish(occ_grid_utils_->GetGrid());
  ros::spinOnce();
}

bool OccGridToCostmapNode::ReceivedMap() {
  return received_map_;
}

void OccGridToCostmapNode::MapCallback(
    const nav_msgs::OccupancyGridConstPtr& occ_grid_msg) {
  ROS_INFO("Received map");

  // Initialize the occupancy grid utils object
  occ_grid_utils_ = boost::make_shared<OccGridUtils>(occ_grid_msg);

  received_map_ = true;
  map_sub_.shutdown();
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "occ_grid_to_costmap_node");

  traxers::OccGridToCostmapNode occ_grid_to_costmap_node;

  // Wait until we receive a map
  ros::Rate r(10);
  while (!occ_grid_to_costmap_node.ReceivedMap() && ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  // Create the costmap
  occ_grid_to_costmap_node.CreateCostmap();

  return 0;
}
