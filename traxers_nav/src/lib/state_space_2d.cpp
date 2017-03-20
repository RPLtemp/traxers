#include "traxers_nav/state_space_2d.h"

namespace traxers {

StateSpace2D::StateSpace2D(const StateSpace2D& state_space) :
    width_(state_space.width_),
    height_(state_space.height_) {
  states_ = state_space.states_;
}

StateSpace2D::StateSpace2D(const std::vector<int8_t>& data, int width, int height) :
    width_(width),
    height_(height) {
  states_ = data;
}

StateSpace2D::~StateSpace2D() {
}

int StateSpace2D::GetHeight() {
  return height_;
}

int StateSpace2D::GetSize() {
  return states_.size();
}

int StateSpace2D::GetWidth() {
  return width_;
}

int8_t StateSpace2D::GetState(int ind) {
  return states_.at(ind);
}

}
