#ifndef NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_H
#define NUMERICAL_INTEGRATOR_RUNGE_KUTTA_INTEGRATOR_H

#include <vector>

#include "eigen3/Eigen/Dense"

/*
StateDerivativeFunction
- NonlinearStateDerivativeFunction
- LinearStateDerivativeFunction

state_derivative_functor = new
NonlinearStateDerivativeFunction<TwoWheelRobotKinematics, 3, 2>(new
TwoWheelRobotKinematics());

  return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
      new SnavelyReprojectionError(observed_x, observed_y)));

*/

namespace numerical_integrator {

// template <int kDimensionOfStateVector,  // dimension of state vector
//           int kDimensionOfInputVector>  // dimension of input vector
// class StateDerivativeFunction {
//  public:
//   StateDerivativeFunction()
//       : dimension_of_state_vector_(kDimensionOfStateVector),
//         dimension_of_input_vector_(kDimensionOfInputVector) {}

//   virtual ~StateDerivativeFunction() {}

//   virtual bool EvaluateStateDerivative(
//       const Eigen::Matrix<kDimensionOfStateVector, 1, double>& X_k,
//       const Eigen::Matrix<kDimensionOfInputVector, 1, double>& u_k);

//  protected:
//   const int GetDimensionOfStateVector() { return dimension_of_state_vector_;
//   } const int GetDimensionOfInputVector() { return
//   dimension_of_input_vector_; }

//  private:
//   int dimension_of_state_vector_;
//   int dimension_of_input_vector_;
// };

template <int kDimensionOfStateVector,  // Dimension of state vector
          int kDimensionOfInputVector>  // Dimension of input vector
class NonlinearDerivativeFunction {
 public:
  explicit NonlinearDerivativeFunction()
      : dimension_of_state_vector_(kDimensionOfStateVector),
        dimension_of_input_vector_(kDimensionOfInputVector) {}
  ~NonlinearDerivativeFunction() {}

  virtual bool EvaluateStateDerivative(
      const Eigen::Matrix<kDimensionOfStateVector, 1, double>& X_k,
      const Eigen::Matrix<kDimensionOfInputVector, 1, double>& u_k) = 0;

 protected:
  const int GetDimensionOfStateVector() { return dimension_of_state_vector_; }
  const int GetDimensionOfInputVector() { return dimension_of_input_vector_; }

 private:
  int dimension_of_state_vector_;
  int dimension_of_input_vector_;
};

class TwoWheeledRobotKinematics
    : public numerical_integrator::NonlinearDerivativeFunction<3, 2> {
 public:
  TwoWheeledRobotKinematics() { GetDimensionOfInputVector(); }

  bool EvaluateStateDerivative(const Eigen::Matrix<3, 1, double>& X_k,
                               const Eigen::Matrix<2, 1, double>& u_k) {
    return true;
  }
};

RungeKutta integrator(new TwoWheeledRobotKinematics());
integrator.IntegrateStateVector(dt, X_k, u_k);
const auto X_k1 = integrator.GetPreviouslyIntegratedStateVector();

// template <typename DerivativeFunctor,
//           int kDimensionOfStateVector,  // Dimension of state vector
//           int kDimensionOfInputVector>  // Dimension of input vector
// class LinearDerivativeFunctor
//     : public SizedDerivativeFunction<kDimensionOfStateVector,
//                                      kDimensionOfInputVector> {
//  public:
//   ~LinearDerivativeFunctor() {}

//   virtual bool Evaluate(
//       const Eigen::Matrix<kDimensionOfStateVector, 1, double>& X_k,
//       const Eigen::Matrix<kDimensionOfInputVector, 1, double>& u_k) = 0;

//  private:
//   Eigen::Matrix<kDimensionOfStateVector, kDimensionOfStateVector, double>
//       system_matrix_;  // A
//   Eigen::Matrix<kDimensionOfStateVector, kDimensionOfInputVector, double>
//       input_matrix_;  // B
// };

class MyDerivativeFunctor : public NonlinearDerivativeFunction<3, 2> {};

template <int state_dimension, int input_dimension>
class RungeKuttaIntegrator {
 public:
  virtual ~RungeKuttaIntegrator() = 0;

 public:
  virtual void Reset() = 0;
  virtual double Integrate(const Eigen::Matrix<state_dimension, 1, double>& X_k,
                           const Eigen::Matrix<input_dimension, 1, double>& u_k,
                           const NonlinearDerivativeFunctor& dX_k,
                           const double dt) = 0;

 protected:
  std::vector<Eigen::Matrix<state_dimension, 1, double>> estimated_state_list_;
};

}  // namespace numerical_integrator

#endif