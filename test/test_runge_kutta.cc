#include <iostream>

#include "src/numerical_integrator/differential_equation.h"
#include "src/numerical_integrator/runge_kutta_integrator.h"
#include "src/numerical_integrator/two_wheeled_robot_differential_equation.h"

using namespace numerical_integrator;

int main() {
  constexpr double l = 0.65;  // [m]
  constexpr double r = 0.12;  // [m]
  std::shared_ptr<DifferentialEquation> differential_equation =
      std::make_shared<SizedDifferentialEquation<
          TwoWheeledRobotDifferentialEquationFunctor, 3, 2>>(
          std::make_shared<TwoWheeledRobotDifferentialEquationFunctor>(l, r));

  RungeKuttaIntegrator<3, 2> integrator(differential_equation);

  Eigen::Matrix<double, 3, 1> X_k;
  Eigen::Matrix<double, 2, 1> u_k;
  Eigen::Matrix<double, 3, 1> dX_k;
  X_k << 4.2, -2.14, 0.91;
  u_k << 1.0, 0.14;

  differential_equation->EvaluateDerivative(X_k.data(), u_k.data(),
                                            dX_k.data());

  const double time_difference = 0.001;
  const auto predicted_X =
      integrator.PredictNextStateByIntegration(X_k, u_k, time_difference);

  std::cout << "X:" << X_k.transpose() << std::endl;
  std::cout << "predicted_X:" << predicted_X.transpose() << std::endl;

  std::cout << "dX_k:" << dX_k.transpose() << std::endl;

  return 0;
}
