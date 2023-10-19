#ifndef NUMERICAL_INTEGRATOR_DIFFERENTIAL_EQUATION_H_
#define NUMERICAL_INTEGRATOR_DIFFERENTIAL_EQUATION_H_

#include <memory>
#include <stdexcept>

#include "eigen3/Eigen/Dense"

namespace numerical_integrator {

class DifferentialEquation {
 public:
  DifferentialEquation() {}
  virtual bool EvaluateDerivative(
      const double* state_vector_ptr, const double* input_vector_ptr,
      double* state_derivative_vector_ptr) const = 0;

  int GetDimensionOfStateVector() const { return dimension_of_state_vector_; }
  int GetDimensionOfInputVector() const { return dimension_of_input_vector_; }

 protected:
  int dimension_of_state_vector_{0};
  int dimension_of_input_vector_{0};
};

template <typename DifferentialEquationFunctor, int kDimensionOfStateVector,
          int kDimensionOfInputVector>
class SizedDifferentialEquation : public DifferentialEquation {
 public:
  SizedDifferentialEquation(const std::shared_ptr<DifferentialEquationFunctor>
                                differential_equation_functor)
      : differential_equation_functor_(differential_equation_functor) {
    if (differential_equation_functor_ == nullptr)
      throw std::runtime_error(
          "Input differential_equation_functor is nullptr.");

    dimension_of_state_vector_ = kDimensionOfStateVector;
    dimension_of_input_vector_ = kDimensionOfInputVector;

    if (dimension_of_state_vector_ < 0)
      throw std::runtime_error(
          "dimension_of_state_vector_ is not positive integer.");
    if (dimension_of_input_vector_ < 0)
      throw std::runtime_error(
          "dimension_of_input_vector_ is not positive integer.");
  }

  bool EvaluateDerivative(const double* state_vector_ptr,
                          const double* input_vector_ptr,
                          double* state_derivative_vector_ptr) const final {
    return (*differential_equation_functor_)(state_vector_ptr, input_vector_ptr,
                                             state_derivative_vector_ptr);
  }

 private:
  const std::shared_ptr<DifferentialEquationFunctor>
      differential_equation_functor_;
};

// template <typename StateDifferentialEquationFunctor,
//           int kDimensionOfStateVector,  // dimension of state vector
//           int kDimensionOfInputVector>  // dimension of input vector
// class SizedDifferentialEquation : public DifferentialEquation {
//  public:
//   explicit SizedDifferentialEquation(
//       const StateDifferentialEquationFunctor* state_derivative_functor)
//       : state_differential_equation_functor_(state_derivative_functor) {}

//   ~SizedDifferentialEquation() final {}

//   bool EvaluateDerivative(const double* const state_vector_ptr,
//                           const double* const input_vector_ptr,
//                           double* state_derivative_vector_ptr) final {
//     (*state_differential_equation_functor_)(state_vector_ptr,
//     input_vector_ptr,
//                                             state_derivative_vector_ptr);
//     return true;
//   }

//  private:
//   const StateDifferentialEquationFunctor*
//   state_differential_equation_functor_;
// };

}  // namespace numerical_integrator

#endif