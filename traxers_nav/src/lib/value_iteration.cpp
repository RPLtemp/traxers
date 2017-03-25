#include "traxers_nav/value_iteration.h"

namespace traxers {

ValueIteration::ValueIteration(double discount, double thresh, int max_iter) :
    discount_(discount),
    thresh_(thresh),
    max_iterations_(max_iter) {
}

ValueIteration::~ValueIteration() {
}

void ValueIteration::AddAction(const Action& action) {
  actions_.push_back(action);
}

void ValueIteration::Init(const StateSpace2D& states,
                          const int goal_state,
                          const std::vector<Action>& actions) {
  if (actions.size() > 0)
    actions_ = actions;

  state_space_ = boost::make_shared<StateSpace2D>(states);

  goal_state_ = goal_state;

  V_.resize(state_space_->GetSize());
  V_.assign(V_.size(), 0.0);
}

void ValueIteration::Run(const bool &save_output) {
  // Verify valid inputs were provided
  assert(actions_.size() > 0);
  assert(state_space_->GetSize() > 0);
  assert(thresh_ > 0.0);

  // Initialize the change between subsequent iterations to be above the
  // threshold
  double delta = std::numeric_limits<double>::max();

  // Run value iteration until we converge
  int iteration = 0;
  while (delta > thresh_ && iteration < max_iterations_) {
    // Copy the cost values from previous iteration
    std::vector<double> V_prev = V_;

    // Loop through the entire state space
    for (int i = 0; i < state_space_->GetSize(); i++) {
      double max_value = std::numeric_limits<double>::lowest();

      // Special case: if the current state is in the wall - should not be
      // there and can't get out of it
      if (state_space_->GetState(i) == 100)
        max_value = 0.0;
      else {
        // Loop through all the possible actions
        for (Action a : actions_) {
          // Apply the action
          int new_state = state_space_->Move(i, a.x, a.y);

          double value;

          if (new_state == -1) {
            value = 0.0;
          }
          else {
            double reward = 0.0;

            if (state_space_->GetState(new_state) == 100)
              reward = -10.0;
            else if (new_state == goal_state_)
              reward = 100.0;

            value = reward + discount_ * V_prev.at(new_state);
          }

          if (value > max_value)
            max_value = value;
        }
      }

      // Assign the new best value
      V_.at(i) = max_value;
    }

    // To check for convergence, we compute the maximum difference between two
    // consecutive iterations
    double max_delta = std::numeric_limits<double>::lowest();
    for (int i = 0; i < V_.size(); i++) {
      double d = fabs(V_.at(i) - V_prev.at(i));

      if (d > max_delta)
        max_delta = d;
    }

    delta = max_delta;

    iteration++;
    //std::cout << "Iteration: " << iteration << std::endl;
    //std::cout << "Max delta: " << delta << std::endl;
  }

  // Save the value function to a color map image
  if (save_output)
    SaveCostmapAsImg(V_, state_space_->GetWidth(), state_space_->GetHeight());
}

void ValueIteration::SetDiscount(double discount) {
  assert(discount > 0.0 && discount <= 1.0);

  discount_ = discount;
}

void ValueIteration::SetMaxIterations(int max_iterations) {
  assert(max_iterations > 0);

  max_iterations_ = max_iterations;
}

void ValueIteration::SetThreshold(double thresh) {
  assert(thresh > 0.0);

  thresh_ = thresh;
}

}
