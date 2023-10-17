#include "src/numerical_integrator/runge_kutta_integrator_fourth_order.h"

namespace numerical_integrator {

template <int state_dimension, int input_dimension>
RungeKuttaIntegratorFourthOrder::RungeKuttaIntegratorFourthOrder(){};

template <int state_dimension, int input_dimension>
RungeKuttaIntegratorFourthOrder::~RungeKuttaIntegratorFourthOrder(){};

void RungeKuttaIntegratorFourthOrder::Reset() {
  state_dimension_ = 0;
  std::vector <
};

double RungeKuttaIntegratorFourthOrder::Integrate(
    const double state_previous, const double difference_previous,
    const double time_difference) {
  return 0;
};

}  // namespace numerical_integrator
