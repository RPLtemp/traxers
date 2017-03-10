#ifndef TRAXERS_OCC_GRID_UTILS_H
#define TRAXERS_OCC_GRID_UTILS_H

#include <nav_msgs/OccupancyGrid.h>

namespace traxers {

enum CELL_STATE: int {
  UNKNOWN = -1,
  FREE = 0,
  OCCUPIED = 100
};

struct Point {
  double x;
  double y;

  // Constructor
  Point(double _x = 0.0, double _y = 0.0) :
      x(_x),
      y(_y) {}

  // Point comparison
  bool operator==(Point p) {
    if (x == p.x && y == p.y)
      return true;
    else
      return false;
  }

  // Point addition
  Point operator+(Point p) {
    return Point(x + p.x, y + p.y);
  }

  // Point subtraction
  Point operator-(Point p) {
    return Point(x - p.x, y - p.y);
  }
};

class OccGridUtils {
  public:
    OccGridUtils(const nav_msgs::OccupancyGridConstPtr& grid) :
        grid_(*grid) {
      origin_x_ = grid_.info.origin.position.x;
      origin_y_ = grid_.info.origin.position.y;

      res_ = grid_.info.resolution;

      width_ = grid_.info.width;
      height_ = grid_.info.height;
    }

    virtual ~OccGridUtils() {}

    int CellIndFromPoint(const Point& pt) {
      if (!IsInsideGrid(pt))
        return CELL_STATE::UNKNOWN;

      int x = (pt.x - origin_x_) / res_;
      int y = (pt.y - origin_y_) / res_;

      return (x + y * width_);
    }

    bool IsInsideGrid(const Point& pt) {
      int x = (pt.x - origin_x_) / res_;
      int y = (pt.y - origin_y_) / res_;

      if (x < 0 || y > 0 || x >= width_ || y >= height_)
        return false;
      else
        return true;
    }

    int GetCellState(const int cell_ind) {
      return grid_.data.at(cell_ind);
    }

    int GetCellState(const Point& pt) {
      int cell_ind = CellIndFromPoint(pt);
      return grid_.data.at(cell_ind);
    }

    nav_msgs::OccupancyGrid GetGrid() {
      return grid_;
    }

    void SetCellState(const int cell_ind, CELL_STATE state) {
      grid_.data.at(cell_ind) = state;
    }

    void SetCellState(const Point& pt, CELL_STATE state) {
      int cell_ind = CellIndFromPoint(pt);
      grid_.data.at(cell_ind) = state;
    }

  private:
    double origin_x_;
    double origin_y_;

    double res_;

    int width_;
    int height_;

    nav_msgs::OccupancyGrid grid_;
};

}

#endif // TRAXERS_OCC_GRID_UTILS_H
