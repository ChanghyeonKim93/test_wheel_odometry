#ifndef NUMERICAL_INTEGRATOR_TWO_WHEELED_ROBOT_DIFFERENTIAL_EQUATION_FUNCTOR_H_
#define NUMERICAL_INTEGRATOR_TWO_WHEELED_ROBOT_DIFFERENTIAL_EQUATION_FUNCTOR_H_

#include "src/numerical_integrator/differential_equation.h"

namespace numerical_integrator {

class TwoWheeledRobotDifferentialEquationFunctor {
 public:
  TwoWheeledRobotDifferentialEquationFunctor() {}

  explicit TwoWheeledRobotDifferentialEquationFunctor(
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

  bool operator()(const double* const state_vector,
                  const double* const input_vector,
                  double* state_derivative_vector) const {
    const double world_to_robot_position_x = state_vector[0];
    const double world_to_robot_position_y = state_vector[1];
    const double world_to_robot_position_psi = state_vector[2];
    const double linear_velocity_in_body_frame = input_vector[0];
    const double angular_velocity_in_body_frame = input_vector[1];

    const double cos_angle = std::cos(world_to_robot_position_psi);
    const double sin_angle = std::sin(world_to_robot_position_psi);

    state_derivative_vector[0] = linear_velocity_in_body_frame * cos_angle;
    state_derivative_vector[1] = linear_velocity_in_body_frame * sin_angle;
    state_derivative_vector[2] = angular_velocity_in_body_frame;

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

}  // namespace numerical_integrator

#endif