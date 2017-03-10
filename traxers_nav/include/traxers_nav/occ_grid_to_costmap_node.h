#ifndef TRAXERS_OCC_GRID_TO_COSTMAP_NODE_H
#define TRAXERS_OCC_GRID_TO_COSTMAP_NODE_H

#include <ros/ros.h>

#include "traxers_nav/occ_grid_utils.h"

namespace traxers {

// Defaults
static constexpr bool kDefaultIsCircular = true;
static constexpr double kDefaultStartX = 0.0;
static constexpr double kDefaultStartY = 0.0;
static constexpr double kDefaultGoalX = 0.0;
static constexpr double kDefaultGoalY = 0.0;

static const std::string kDefaultGridPubTopic = "occ_grid";
static const std::string kDefaultMapSubTopic = "map";

class OccGridToCostmapNode {
  public:
    OccGridToCostmapNode();
    virtual ~OccGridToCostmapNode();

    void CreateCostmap();

    void PublishGrid();

    bool ReceivedMap() { return received_map_; }

  private:
    ros::NodeHandle nh_;

    ros::Publisher grid_pub_;
    ros::Subscriber map_sub_;

    bool is_circular_;
    bool received_map_;

    Point start_pt_;
    Point goal_pt_;

    boost::shared_ptr<OccGridUtils> occ_grid_utils_;

    void MapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);
};

}

#endif // TRAXERS_OCC_GRID_TO_COSTMAP_NODE_H
