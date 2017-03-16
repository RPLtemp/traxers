#ifndef TRAXERS_STATE_SPACE_2D_H
#define TRAXERS_STATE_SPACE_2D_H

#include <stdint.h>
#include <vector>

namespace traxers {

class StateSpace2D {
  public:
    StateSpace2D(const StateSpace2D& state_space);
    StateSpace2D(const std::vector<int8_t> &data, int width, int height);
    virtual ~StateSpace2D();

    int getHeight();

    int GetSize();

    int GetWidth();

  private:
    int width_;
    int height_;

    std::vector<int8_t> states_;
};

}

#endif // TRAXERS_STATE_SPACE_2D_H
