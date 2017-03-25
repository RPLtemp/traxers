#include "traxers_nav/costmap_vis.h"

namespace traxers {

cv::Mat CostmapToImg(const std::vector<double>& costmap, int width,
    int height, bool write_file, const std::string& outfile) {
  assert(costmap.size() == height * width);

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

  // Write the image file to disk if required
  if (write_file) {
    // Set the output file name to default if one is not provided
    const std::string filename = outfile.empty() ? kDefaultCostmapOutfileName : outfile;

    cv::imwrite(filename, img_cm);
  }

  return img_cm;
}

cv::Mat TrajToImg(const cv::Mat& img_cm, const std::vector<int> &traj,
                  bool write_file, const std::string& outfile) {
  // Copy the input costmap image into the output trajectory image
  cv::Mat img_traj;
  img_cm.copyTo(img_traj);

  // Extract the width of the image
  int width = img_traj.cols;

  // Loop through the trajectory and draw lines between consecutive points
  for (int i = 1; i < traj.size(); i++) {
    int ind_1 = traj.at(i);
    int ind_0 = traj.at(i - 1);

    int x_1 = ind_1 % width;
    int y_1 = ind_1 / width;

    int x_0 = ind_0 % width;
    int y_0 = ind_0 / width;

    cv::line(img_traj, cv::Point(x_1, y_1), cv::Point(x_0, y_0), cv::Scalar(255));
  }

  // Write the image file to disk if required
  if (write_file) {
    // Set the output file name to default if one is not provided
    const std::string filename = outfile.empty() ? kDefaultTrajOutfileName : outfile;

    cv::imwrite(filename, img_traj);
  }

  return img_traj;
}

}
