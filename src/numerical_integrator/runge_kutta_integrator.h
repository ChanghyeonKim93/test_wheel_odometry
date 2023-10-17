#ifndef NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_H
#define NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_H

#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"

/*
StateDerivativeFunction
- NonlinearStateDerivativeFunction
- LinearStateDerivativeFunction
*/

/*
numerical_integrator::RungeKuttaFourthOrderIntegrator
  runge_kutta_fourth_order_integrator(new StateDerivativeFunction<3, 2>(new
  TwoWheeledKinematicsFunctor()));


  runge_kutta_fourth_order_integrator.Integrate(dt, X_k, u_k);

*/

namespace numerical_integrator {

template <typename StateDerivativeFunctor,
          int kDimensionOfStateVector,  // dimension of state vector
          int kDimensionOfInputVector>  // dimension of input vector
class SizedStateDerivativeFunction {
 public:
  explicit SizedStateDerivativeFunction(
      const StateDerivativeFunctor* state_derivative_functor)
      : state_derivative_functor_(state_derivative_functor) {}

  ~SizedStateDerivativeFunction() {}

  bool EvaluateStateDerivative(
      const Eigen::Matrix<double, kDimensionOfStateVector, 1>& X_k,
      const Eigen::Matrix<double, kDimensionOfInputVector, 1>& u_k,
      Eigen::Matrix<double, kDimensionOfStateVector, 1>*
          state_derivative_vector) {
    state_derivative_functor_->EvaluateStateDerivative(
        X_k.data(), u_k.data(), state_derivative_vector->data());

    std::cout << state_derivative_vector->transpose() << std::endl;
    return true;
  }

 private:
  const StateDerivativeFunctor* state_derivative_functor_;
};

}  // namespace numerical_integrator

#endif