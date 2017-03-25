#ifndef TRAXERS_VALUE_ITERATION_H
#define TRAXERS_VALUE_ITERATION_H

#include <assert.h>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "traxers_nav/costmap_vis.h"
#include "traxers_nav/state_space_2d.h"

namespace traxers {

// Default values
static constexpr double kDefaultDiscount = 0.99;
static constexpr double kDefaultThresh = 0.000001;
static constexpr int kDefaultMaxIterations = 500;

struct Action {
  int x;
  int y;

  Action(int _x = 0, int _y = 0) :
      x(_x),
      y(_y) {}
};

class ValueIteration {
  public:
    ValueIteration(double discount = kDefaultDiscount, double thresh = kDefaultThresh, int max_iter = kDefaultMaxIterations);
    virtual ~ValueIteration();

    void AddAction(const Action& action);

    void Init(const StateSpace2D& states, const int goal_state, const std::vector<Action>& actions = std::vector<Action>());

    void Run(const bool& save_output = true);

    void SetDiscount(double discount);

    void SetMaxIterations(int max_iterations);

    void SetThreshold(double thresh);

  private:
    double discount_;
    double thresh_;

    int goal_state_;
    int max_iterations_;

    std::vector<Action> actions_;

    std::vector<double> V_;

    boost::shared_ptr<StateSpace2D> state_space_;
};

}

#endif // TRAXERS_VALUE_ITERATION_H
