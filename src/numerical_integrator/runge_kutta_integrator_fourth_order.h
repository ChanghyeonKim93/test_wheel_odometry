#ifndef NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_FOURTH_ORDER_H
#define NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_FOURTH_ORDER_H

#include <iostream>

#include "src/numerical_integrator/runge_kutta_integrator.h"

namespace numerical_integrator {

template <int state_dimension, int input_dimension>
class RungeKuttaIntegratorFourthOrder : public RungeKuttaIntegrator {
 public:
  RungeKuttaIntegratorFourthOrder();
  ~RungeKuttaIntegratorFourthOrder() final;

 public:
  void Reset() final;
  double Integrate(const Eigen::Matrix<state_dimension, 1, double>& X_k,
                   const Eigen::Matrix<input_dimension, 1, double>& u_k,
                   const NonlinearDerivativeFunctor& dX_k,
                   const double dt) final;
};

}  // namespace numerical_integrator

#endif