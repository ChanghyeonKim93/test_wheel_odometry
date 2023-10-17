#include <iostream>

#include "src/numerical_integrator/runge_kutta_integrator.h"

using namespace numerical_integrator;

class Kinematics {
 public:
  virtual bool EvaluateStateDerivative(
      const double* const current_state_vector,
      const double* const current_input_vector,
      double* current_state_derivative_vector) const = 0;
};

class TwoWheeledRobotKinematics : public Kinematics {
 public:
  explicit TwoWheeledRobotKinematics(
      const double length_between_wheel_centers_in_meters,
      const double radius_of_wheel_in_meters)
      : length_between_wheel_centers_in_meters_(
            length_between_wheel_centers_in_meters),
        radius_of_wheel_in_meters_(radius_of_wheel_in_meters),
        kinematics_matrix_{} {
    const auto& r = radius_of_wheel_in_meters_;
    const auto& L = length_between_wheel_centers_in_meters_;
    kinematics_matrix_ << r * 0.5, r * 0.5, r / L, -r / L;
    inverse_kinematics_matrix_ = kinematics_matrix_.inverse();
  }

  bool EvaluateStateDerivative(
      const double* const current_state_vector,
      const double* const current_input_vector,
      double* current_state_derivative_vector) const final {
    const double world_to_robot_position_x = current_state_vector[0];
    const double world_to_robot_position_y = current_state_vector[1];
    const double world_to_robot_position_psi = current_state_vector[2];
    const double body_linear_velocity = current_input_vector[0];
    const double body_angular_velocity = current_input_vector[1];

    const double cos_angle = std::cos(world_to_robot_position_psi);
    const double sin_angle = std::sin(world_to_robot_position_psi);

    current_state_derivative_vector[0] = body_linear_velocity * cos_angle;
    current_state_derivative_vector[1] = body_linear_velocity * sin_angle;
    current_state_derivative_vector[2] = body_angular_velocity;

    return true;
  }

 public:
  const Eigen::Matrix<double, 2, 2>& GetKinematicsMatrix() const {
    return kinematics_matrix_;
  }

  const Eigen::Matrix<double, 2, 2>& GetInverseKinematicsMatrix() const {
    return inverse_kinematics_matrix_;
  }

 private:
  double length_between_wheel_centers_in_meters_;
  double radius_of_wheel_in_meters_;
  Eigen::Matrix<double, 2, 2>
      kinematics_matrix_;  // wheel angular velocities to body linear and
                           // angular velocities
  Eigen::Matrix<double, 2, 2> inverse_kinematics_matrix_;
};

int main() {
  int a = 0;

  const double L = 0.65;  // [m]
  const double r = 0.12;  // [m]
  auto de = SizedStateDerivativeFunction<TwoWheeledRobotKinematics, 3, 2>(
      new TwoWheeledRobotKinematics(L, r));

  Eigen::Matrix<double, 3, 1> X_k;
  Eigen::Matrix<double, 2, 1> u_k;
  Eigen::Matrix<double, 3, 1> dX_k;
  X_k << 4.2, -2.14, 0.91;
  u_k << 1.0, 0.14;

  de.EvaluateStateDerivative(X_k, u_k, &dX_k);

  return 0;
}