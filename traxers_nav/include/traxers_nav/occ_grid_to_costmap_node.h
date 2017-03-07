#ifndef TRAXERS_OCC_GRID_TO_COSTMAP_NODE_H
#define TRAXERS_OCC_GRID_TO_COSTMAP_NODE_H

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace traxers {

// Constants
static const std::string kMapSubTopic = "map";

class OccGridToCostmapNode {
  public:
    OccGridToCostmapNode();
    virtual ~OccGridToCostmapNode();

  private:
    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;

    void MapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);
};

}

#endif // TRAXERS_OCC_GRID_TO_COSTMAP_NODE_H
