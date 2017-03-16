#ifndef TRAXERS_COSTMAP_VIS_H
#define TRAXERS_COSTMAP_VIS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace traxers {

// Constants
static const std::string kDefaultOutfileName = "./costmap.jpg";

void SaveCostmapAsImg(const std::vector<double>& costmap, int width,
                      int height, const std::string& outfile = "");

}

#endif // TRAXERS_COSTMAP_VIS_H
