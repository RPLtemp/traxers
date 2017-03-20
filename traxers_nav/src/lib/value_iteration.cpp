#include "traxers_nav/value_iteration.h"

namespace traxers {

ValueIteration::ValueIteration(double thresh) :
    thresh_(thresh) {
}

ValueIteration::~ValueIteration() {
}

void ValueIteration::AddAction(const Action& action) {
  actions_.push_back(action);
}

void ValueIteration::Init(const StateSpace2D& states, const std::vector<Action>& actions) {
  if (actions.size() > 0)
    actions_ = actions;

  state_space_ = boost::make_shared<StateSpace2D>(states);

  V_.resize(state_space_->GetSize());
  V_.assign(V_.size(), 0.0);
}

void ValueIteration::Run(const bool &save_output) {
  // Verify valid inputs were provided
  assert(actions_.size() > 0);
  assert(state_space_->GetSize() > 0);
  assert(thresh_ > 0.0);

  // Initialize the change between subsequent iterations to be above the threshold
  double delta = 1.0;

  // Run value iteration until we converge
  while (delta < thresh_) {
    // Loop through the entire state space
    for (int i = 0; i < state_space_->GetSize(); i++) {

    }

    delta = 0.0;
  }

  // Save the value function to a color map image
  if (save_output)
    SaveCostmapAsImg(V_, state_space_->GetWidth(), state_space_->GetHeight());
}

void ValueIteration::SetThreshold(double thresh) {
  thresh_ = thresh;
}

}
