#ifndef TRAXERS_COSTMAP_VIS_H
#define TRAXERS_COSTMAP_VIS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#if CV_MAJOR_VERSION == 3
  #include <opencv2/imgproc/imgproc.hpp>
#elif CV_MAJOR_VERSION == 2
  #include <opencv2/contrib/contrib.hpp>
#endif

namespace traxers {

// Constants
static const std::string kDefaultCostmapOutfileName = "./costmap.jpg";
static const std::string kDefaultTrajOutfileName = "./traj.jpg";

cv::Mat CostmapToImg(const std::vector<double>& costmap, int width, int height,
                     bool write_file = false, const std::string& outfile = "");

cv::Mat TrajToImg(const cv::Mat& img_cm, const std::vector<int>& traj,
                  bool write_file = false, const std::string& outfile = "");

}

#endif // TRAXERS_COSTMAP_VIS_H
