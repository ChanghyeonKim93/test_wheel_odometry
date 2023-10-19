#ifndef NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_H
#define NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_H

#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "src/numerical_integrator/differential_equation.h"

namespace numerical_integrator {

template <int kDimensionOfStateVector, int kDimensionOfInputVector>
class RungeKuttaIntegrator {
 public:
  RungeKuttaIntegrator(
      const std::shared_ptr<DifferentialEquation>& differential_equation);
  ~RungeKuttaIntegrator();

  const Eigen::Matrix<double, kDimensionOfStateVector, 1>
  PredictNextStateByIntegration(
      const Eigen::Matrix<double, kDimensionOfStateVector, 1>&
          current_state_vector,
      const Eigen::Matrix<double, kDimensionOfInputVector, 1>&
          current_input_vector,
      const double time_difference_in_second);

 private:
  std::shared_ptr<DifferentialEquation> differential_equation_{nullptr};
  int dimension_of_state_vector_{0};
  int dimension_of_input_vector_{0};
};

template <int kDimensionOfStateVector, int kDimensionOfInputVector>
RungeKuttaIntegrator<kDimensionOfStateVector, kDimensionOfInputVector>::
    RungeKuttaIntegrator(
        const std::shared_ptr<DifferentialEquation>& differential_equation) {
  if (differential_equation == nullptr)
    throw std::runtime_error("differential_equation is nullptr");

  differential_equation_ = differential_equation;

  dimension_of_state_vector_ = kDimensionOfStateVector;
  if (dimension_of_state_vector_ == 0)
    throw std::runtime_error("dimension_of_state_vector_ == 0");
  if (dimension_of_state_vector_ !=
      differential_equation_->GetDimensionOfStateVector())
    throw std::runtime_error(
        "dimension_of_state_vector_ "
        "!= differential_equation_->GetDimensionOfStateVector()");

  dimension_of_input_vector_ = kDimensionOfInputVector;
  if (dimension_of_input_vector_ == 0)
    throw std::runtime_error("dimension_of_input_vector_ == 0");
  if (dimension_of_input_vector_ !=
      differential_equation_->GetDimensionOfInputVector())
    throw std::runtime_error(
        "dimension_of_input_vector_ "
        "!= differential_equation_->GetDimensionOfInputVector()");
}

template <int kDimensionOfStateVector, int kDimensionOfInputVector>
RungeKuttaIntegrator<kDimensionOfStateVector,
                     kDimensionOfInputVector>::~RungeKuttaIntegrator() {}

template <int kDimensionOfStateVector, int kDimensionOfInputVector>
const Eigen::Matrix<double, kDimensionOfStateVector, 1>
RungeKuttaIntegrator<kDimensionOfStateVector, kDimensionOfInputVector>::
    PredictNextStateByIntegration(
        const Eigen::Matrix<double, kDimensionOfStateVector, 1>&
            current_state_vector,
        const Eigen::Matrix<double, kDimensionOfInputVector, 1>&
            current_input_vector,
        const double time_difference_in_second) {
  // Calculate state derivative vector
  Eigen::Matrix<double, kDimensionOfStateVector, 1>
      current_state_derivative_vector;
  differential_equation_->EvaluateDerivative(
      current_state_vector.data(), current_input_vector.data(),
      current_state_derivative_vector.data());

  // Calculate k1,k2,k3,k4
  const Eigen::Matrix<double, kDimensionOfStateVector, 1>& k1 =
      current_state_derivative_vector;

  Eigen::Matrix<double, kDimensionOfStateVector, 1> k2;
  const Eigen::Matrix<double, kDimensionOfStateVector, 1> predicted_state_1 =
      current_state_vector + 0.5 * time_difference_in_second * k1;
  differential_equation_->EvaluateDerivative(
      predicted_state_1.data(), current_input_vector.data(), k2.data());

  Eigen::Matrix<double, kDimensionOfStateVector, 1> k3;
  const Eigen::Matrix<double, kDimensionOfStateVector, 1> predicted_state_2 =
      current_state_vector + 0.5 * time_difference_in_second * k2;
  differential_equation_->EvaluateDerivative(
      predicted_state_2.data(), current_input_vector.data(), k3.data());

  Eigen::Matrix<double, kDimensionOfStateVector, 1> k4;
  const Eigen::Matrix<double, kDimensionOfStateVector, 1> predicted_state_3 =
      current_state_vector + time_difference_in_second * k3;
  differential_equation_->EvaluateDerivative(
      predicted_state_3.data(), current_input_vector.data(), k4.data());

  // Predict next state vector
  static constexpr double kInverseSix = 1.0 / 6.0;
  Eigen::Matrix<double, kDimensionOfStateVector, 1> predicted_next_state_vector;
  predicted_next_state_vector =
      current_state_vector +
      kInverseSix * time_difference_in_second * (k1 + 2.0 * (k2 + k3) + k4);

  return predicted_next_state_vector;
}

}  // namespace numerical_integrator

#endif