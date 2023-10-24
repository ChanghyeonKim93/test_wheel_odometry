#include "wheel_imu_error_state_kalman_filter/wheel_imu_error_state_kalman_filter.h"

#include <iostream>

namespace utility {
namespace filter {
namespace error_state_kalman_filter {

WheelImuErrorStateKalmanFilter::WheelImuErrorStateKalmanFilter(
    const Parameters& parameters)
    : is_initialized_(false),
      parameters_(parameters) {  // initialize error covariance matrix

  // Initialize Error State
  error_state_.SetZero();

  // Initialize Nominal State
  const auto& imu_bias = parameters_.initial_bias.imu;
  nominal_state_.SetAccXYBias({imu_bias.acc_x_bias, imu_bias.acc_y_bias});
  nominal_state_.SetGyroZBias(imu_bias.gyro_z_bias);

  // Initialize Error Covariance Matrix
  error_state_covariance_.SetCovarianceMatrix(Mat99::Identity() * 0.5);

  // Initialize Error State Process Noises
  const auto& initial_error_state_process_noise =
      parameters_.noise.error_state_process;
  error_state_process_noise_.SetPositionErrorProcessNoise(
      {initial_error_state_process_noise.position_x,
       initial_error_state_process_noise.position_y});
  error_state_process_noise_.SetVelocityErrorProcessNoise(
      {initial_error_state_process_noise.velocity_x,
       initial_error_state_process_noise.velocity_y});
  error_state_process_noise_.SetYawErrorProcessNoise(
      initial_error_state_process_noise.yaw);
  error_state_process_noise_.SetYawRateErrorProcessNoise(
      initial_error_state_process_noise.yaw_rate);
  error_state_process_noise_.SetAccXYBiasErrorProcessNoise(
      {initial_error_state_process_noise.acc_bias_x,
       initial_error_state_process_noise.acc_bias_y});
  error_state_process_noise_.SetAGyroZBiasErrorProcessNoise(
      initial_error_state_process_noise.gyro_bias_z);

  // Initialize Imu Measurement Noises
  const auto& imu_noise = parameters_.noise.measurement.imu;
  imu_measurement_noise_.SetAccXYNoise(imu_noise.acc_x, imu_noise.acc_y);
  imu_measurement_noise_.SetGyroZNoise(imu_noise.gyro_z);

  // Initialize Wheel Encoder Measurement Noises
  const auto& wheel_encoder_noise = parameters_.noise.measurement.wheel_encoder;
  wheel_encoder_measurement_noise_.SetLeftRightAngularRatesNoises(
      wheel_encoder_noise.left_angular_rate,
      wheel_encoder_noise.right_angular_rate);

  // Initialize wheel odometer kinematics
  const double l = parameters_.wheeled_robot_properties.distance_between_wheels;
  const double r = parameters_.wheeled_robot_properties.wheel_radius_in_meter;
  B_ << r / 2.0, r / 2.0, -r / l, r / l;
  iB_ << 2.0 / r, -l / r, 2.0 / r, l / r;

  std::cerr << "Wheel-IMU error state Kalman filter is constructed.\n";
}

void WheelImuErrorStateKalmanFilter::Reset() {
  // TODO(@): implement reset
}

// ## Get `Exp(F0*dt)` matrix
Mat99 WheelImuErrorStateKalmanFilter::CalculateErrorStateTransitionMatrix(
    const double previous_timestamp, const NominalState& previous_nominal_state,
    const double current_timestamp, const ImuMeasurement& imu_measurement) {
  const double dt = current_timestamp - previous_timestamp;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;

  const double yaw = previous_nominal_state.GetWorldYaw();
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Mat22 R_world_to_imu;
  R_world_to_imu << cy, -sy, sy, cy;

  const auto& am = imu_measurement.GetAccelerationVector();
  const auto& ba = nominal_state_.GetAccXYBias();
  const Vec2 acc_compensated(am - ba);
  Vec2 acc_compensated_cross;
  acc_compensated_cross << acc_compensated.y(), -acc_compensated.x();

  // clang-format off
  const auto& R = R_world_to_imu;
  const auto& ax = acc_compensated_cross;
  Mat99 exp_F0dt{Mat99::Identity()}; 
  exp_F0dt << //
    I22, dt*I22, -0.5*R*ax*dt2, O21, -0.5*R*dt2, -1.0/6.0*R*ax*dt3,//
    O22,    I22,      -R*ax*dt, O21,      -R*dt, -1.0/2.0*R*ax*dt3,//
    O12,    O12,             1,   0,        O12,                dt,//
    O12,    O12,           O11,   1,        O12,                 0,//
    O22,    O22,           O21, O21,        I22,               O21,//
    O12,    O12,             0,   0,        O12,                 1;
  // clang-format on

  return exp_F0dt;
}

void WheelImuErrorStateKalmanFilter::PredictErrorStateByImuDeadReckoning(
    const double previous_timestamp, const ErrorState& previous_error_state,
    const NominalState& previous_nominal_state,
    const ErrorStateCovariance& previous_error_state_covariance,
    const double current_timestamp, const ImuMeasurement& imu_measurement,
    ErrorState* predicted_error_state,
    ErrorStateCovariance* predicted_error_state_covariance) {
  const Mat99& exp_F0dt = CalculateErrorStateTransitionMatrix(
      previous_timestamp, previous_nominal_state, current_timestamp,
      imu_measurement);
  const Vec9& predicted_error_state_vector =
      exp_F0dt * error_state_.GetErrorStateVector();

  ErrorState predicted_error_state_in_function(current_timestamp,
                                               predicted_error_state_vector);
  predicted_error_state->SetErrorState(predicted_error_state_in_function);

  const Mat99& Q = error_state_process_noise_.GetProcessNoiseMatrix();
  const Mat99& P_prev = previous_error_state_covariance.GetCovarianceMatrix();
  const Mat99& P_predicted = exp_F0dt * P_prev * exp_F0dt.transpose() + Q;
  predicted_error_state_covariance->SetCovarianceMatrix(P_predicted);
}

void WheelImuErrorStateKalmanFilter::PredictNominalStateByImuDeadReckoning(
    const double previous_timestamp, const NominalState& previous_nominal_state,
    const double current_timestamp, const ImuMeasurement& imu_measurement,
    NominalState* predicted_nominal_state) {
  const double dt = current_timestamp - previous_timestamp;

  const double yaw = previous_nominal_state.GetWorldYaw();
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Mat22 R_world_to_imu;
  R_world_to_imu << cy, -sy, sy, cy;

  const auto& am = imu_measurement.GetAccelerationVector();
  const auto gz = imu_measurement.GetYawRate();

  const auto& v = nominal_state_.GetWorldVelocity();
  const auto& ba = nominal_state_.GetAccXYBias();
  const auto& bg = nominal_state_.GetGyroZBias();

  Vec9 derivative_of_previous_nominal_state;
  derivative_of_previous_nominal_state << v, am - ba, gz - bg, O21, 0;

  // TODO(@): employ RK4
  // Note: below is the Euler integration.
  const auto& previous_nominal_state_vector =
      nominal_state_.GetFullStateVector();
  const Vec9& predicted_nominal_state_vector =
      previous_nominal_state_vector + dt * derivative_of_previous_nominal_state;

  NominalState predicted_nominal_state_in_function(
      current_timestamp, predicted_nominal_state_vector);
  predicted_nominal_state->SetNominalState(predicted_nominal_state_in_function);
}

void WheelImuErrorStateKalmanFilter::
    EstimateNominalStateByWheelEncoderMeasurement(
        const NominalState& predicted_nominal_state,
        const ErrorState& predicted_error_state,
        const ErrorStateCovariance& predicted_error_state_covariance,
        const WheelEncoderMeasurement& wheel_encoder_measurement,
        ErrorState* reset_error_state, NominalState* estimated_nominal_state) {
  const Mat99& P_predicted =
      predicted_error_state_covariance.GetCovarianceMatrix();
  const Mat22& R = wheel_encoder_measurement_noise_
                       .GetLeftRightAngularRatesCovarianceMatrix();
  const Vec2& z = wheel_encoder_measurement.GetLeftAndRightAngularRateVector();

  const double yaw = predicted_nominal_state.GetWorldYaw();
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Mat29 Hback;
  // clang-format off
  Hback << 
    O12, cy, sy, 0, 0, O12, 0,  //
    O12,    O12, 0, 1, O12, 0;
  // clang-format on

  const Mat29& H = iB_ * Hback;
  const Mat92& K = P_predicted * H.transpose() *
                   (H * P_predicted * H.transpose() + R).inverse();
  const Mat99& P_estimated =
      (I99 - K * H) * P_predicted * (I99 - K * H).transpose() +
      K * R * K.transpose();
  const Vec9& dX_estimated =
      predicted_error_state.GetErrorStateVector() +
      K * (z - H * predicted_nominal_state.GetFullStateVector());

  const Vec9& X_estimated =
      predicted_nominal_state.GetFullStateVector() + dX_estimated;

  *reset_error_state =
      ErrorState(wheel_encoder_measurement.GetTimestamp(), Vec9::Zero());
  *estimated_nominal_state =
      NominalState(wheel_encoder_measurement.GetTimestamp(), X_estimated);
}

const NominalState& WheelImuErrorStateKalmanFilter::GetNominalState() const {
  return nominal_state_;
}

const ErrorState& WheelImuErrorStateKalmanFilter::GetErrorState() const {
  return error_state_;
}

const ErrorStateCovariance&
WheelImuErrorStateKalmanFilter::GetErrorStateCovariance() const {
  return error_state_covariance_;
}

}  // namespace error_state_kalman_filter
}  // namespace filter
}  // namespace utility