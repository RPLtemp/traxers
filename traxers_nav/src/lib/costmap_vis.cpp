#include "traxers_nav/costmap_vis.h"

namespace traxers {

void SaveCostmapAsImg(const std::vector<double>& costmap, int width,
    int height, const std::string& outfile) {
  assert(costmap.size() == height * width);

  // Set the output file name to default if one is not provided
  const std::string filename = outfile.empty() ? kDefaultOutfileName : outfile;

  // Initialize the grayscale image
  cv::Mat img(width, height, CV_8UC1);

  // Extract the maximum cost for scaling everything between 0 and 255
  double max_value = *std::max_element(costmap.begin(), costmap.end());

  // Map the vector of cost values to a grayscale image matrix
  for (int i = 0; i < costmap.size(); i++) {
    int x = i % width;
    int y = i / width;

    assert(x >= 0 && y >= 0);
    assert(x < width && y < height);

    img.at<uchar>(x, y) = 255 * (costmap.at(i) / max_value);
  }

  // Apply the color map for visualization
  cv::Mat img_cm;
  cv::applyColorMap(img, img_cm, cv::COLORMAP_JET);

  // Write the image file to disk
  cv::imwrite(filename, img_cm);
}

}
