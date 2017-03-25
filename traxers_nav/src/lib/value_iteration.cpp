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
}

void ValueIteration::Run(std::vector<Action> &optimal_actions, std::vector<double> &V) {
  // Verify valid inputs were provided
  assert(actions_.size() > 0);
  assert(state_space_->GetSize() > 0);
  assert(thresh_ > 0.0);

  V.resize(state_space_->GetSize());
  V.assign(V.size(), 0.0);

  // The vector of optimal actions should be the same size as the state space
  optimal_actions.resize(state_space_->GetSize());

  // Initialize the change between subsequent iterations to be above the
  // threshold
  double delta = std::numeric_limits<double>::max();

  // Run value iteration until we converge
  int iteration = 0;
  while (delta > thresh_ && iteration < max_iterations_) {
    // Copy the cost values from previous iteration
    std::vector<double> V_prev = V;

    // Loop through the entire state space
    for (int i = 0; i < state_space_->GetSize(); i++) {
      double max_value = std::numeric_limits<double>::lowest();

      // Initialize the best action as strictly forward motion
      Action best_action(3, 0);

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

          if (value > max_value) {
            max_value = value;
            best_action = a;
          }
        }
      }

      // Assign the new best value
      V.at(i) = max_value;

      // Save the best action for this state
      optimal_actions.at(i) = best_action;
    }

    // To check for convergence, we compute the maximum difference between two
    // consecutive iterations
    double max_delta = std::numeric_limits<double>::lowest();
    for (int i = 0; i < V.size(); i++) {
      double d = fabs(V.at(i) - V_prev.at(i));

      if (d > max_delta)
        max_delta = d;
    }

    delta = max_delta;

    iteration++;

    std::cout << "Iteration: " << iteration << std::endl;
    std::cout << "Max delta: " << delta << std::endl;
  }
}

void ValueIteration::Plan(const std::vector<Action>& optimal_actions,
                          const int start_state, const int goal_state,
                          std::vector<int>& traj) {
  // Clear the trajectory
  traj.clear();

  // Begin at the start state
  traj.push_back(start_state);
  int current_state = start_state;

  // Follow the optimal action until goal is reached
  bool reached_goal = false;
  while (!reached_goal) {
    // Extract the best action at current state
    Action best_action = optimal_actions.at(current_state);

    // Apply the action
    int new_state = state_space_->Move(current_state,
                                       best_action.x,
                                       best_action.y);

    // Add the new state to the trajectory
    traj.push_back(new_state);

    // Update the current state
    current_state = new_state;

    // Check if we have reached the goal
    if (current_state == goal_state)
      reached_goal = true;
  }
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
