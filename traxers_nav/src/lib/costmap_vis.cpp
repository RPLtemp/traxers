#include "traxers_nav/costmap_vis.h"

namespace traxers {

void SaveCostmapAsImg(const std::vector<double>& costmap, int width,
    int height, const std::string& outfile) {
  // Set the output file name to default if one is not provided
  const std::string filename = outfile.empty() ? kDefaultOutfileName : outfile;
}

}
