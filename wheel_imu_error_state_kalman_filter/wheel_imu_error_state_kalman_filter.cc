#include "wheel_imu_error_state_kalman_filter/wheel_imu_error_state_kalman_filter.h"

#include <iostream>

namespace utility {
namespace filter {
namespace error_state_kalman_filter {

WheelImuErrorStateKalmanFilter::WheelImuErrorStateKalmanFilter(
    const Parameters& parameters)
    : parameters_(parameters) {  // initialize error covariance matrix
  // Initialize Error State
  error_state_.SetZero();

  // Initialize Nominal State
  const auto& imu_bias = parameters_.initial_bias.imu;
  nominal_state_.SetAccXYBias({imu_bias.ba_x, imu_bias.ba_y});
  nominal_state_.SetGyroZBias(imu_bias.bg_z);

  // Initialize Error Covariance Matrix
  error_state_covariance_.SetCovarianceMatrix(Mat99::Identity() * 10.5);

  // Initialize Error State Process Noises
  const auto& initial_error_state_process_noise =
      parameters_.noise.error_state_process;
  error_state_process_noise_.SetPositionErrorProcessNoise(
      {initial_error_state_process_noise.px,
       initial_error_state_process_noise.py});
  error_state_process_noise_.SetVelocityErrorProcessNoise(
      {initial_error_state_process_noise.vx,
       initial_error_state_process_noise.vy});
  error_state_process_noise_.SetYawErrorProcessNoise(
      initial_error_state_process_noise.yaw);
  error_state_process_noise_.SetYawRateErrorProcessNoise(
      initial_error_state_process_noise.yaw_rate);
  error_state_process_noise_.SetAccXYBiasErrorProcessNoise(
      {initial_error_state_process_noise.ba_x,
       initial_error_state_process_noise.ba_y});
  error_state_process_noise_.SetGyroZBiasErrorProcessNoise(
      initial_error_state_process_noise.bg_z);

  // Initialize Imu Measurement Noises
  const auto& imu_noise = parameters_.noise.measurement.imu;
  imu_measurement_noise_.SetAccXYNoise(imu_noise.ax, imu_noise.ay);
  imu_measurement_noise_.SetGyroZNoise(imu_noise.gz);

  // Initialize Wheel Encoder Measurement Noises
  const auto& wheel_encoder_noise = parameters_.noise.measurement.wheel_encoder;
  wheel_encoder_measurement_noise_.SetLeftAndRightAngularRateNoises(
      wheel_encoder_noise.left_angular_rate,
      wheel_encoder_noise.right_angular_rate);

  // Initialize wheel odometer kinematics
  l_ = parameters_.wheeled_robot_properties.distance_between_wheels;
  r_ = parameters_.wheeled_robot_properties.wheel_radius_in_meter;

  std::cerr << "Wheel-IMU error state Kalman filter is constructed.\n";
}

void WheelImuErrorStateKalmanFilter::Reset() {
  // TODO(@): implement reset
}

void WheelImuErrorStateKalmanFilter::SetInitialNominalState(
    const double current_timestamp, const Vec9& initial_state_vector) {
  nominal_state_ = NominalState(current_timestamp, initial_state_vector);
}

void WheelImuErrorStateKalmanFilter::
    PredictNominalAndErrorStatesByImuMeasurement(
        const double current_timestamp, const ImuMeasurement& imu_measurement) {
  //   std::cerr << "imu_measurement:" << imu_measurement.GetAccerelationX() <<
  //   ", "
  //             << imu_measurement.GetAccerelationY() << ", "
  //             << imu_measurement.GetYawRate() << std::endl;

  NominalState predicted_nominal_state;
  PredictNominalStateByImuDeadReckoning(
      nominal_state_.GetTimestamp(), nominal_state_, current_timestamp,
      imu_measurement, &predicted_nominal_state);

  ErrorState predicted_error_state;
  ErrorStateCovariance predicted_error_state_covariance;
  PredictErrorStateByImuDeadReckoning(
      nominal_state_.GetTimestamp(), error_state_, nominal_state_,
      error_state_covariance_, current_timestamp, imu_measurement,
      &predicted_error_state, &predicted_error_state_covariance);

  nominal_state_ = predicted_nominal_state;
  error_state_ = predicted_error_state;
  error_state_covariance_ = predicted_error_state_covariance;

  std::cout << "   predicted_error_state(after): "
            << error_state_.GetErrorStateVector().transpose() << std::endl;
  std::cout << "   predicted_error_state cov(after):\n"
            << error_state_covariance_.GetCovarianceMatrix() << std::endl;
}

void WheelImuErrorStateKalmanFilter::
    EstimateNominalStateByWheelEncoderMeasurement(
        const double current_timestamp,
        const WheelEncoderMeasurement& wheel_encoder_measurement) {
  NominalState estimated_nominal_state;
  ErrorState reset_error_state;
  ErrorStateCovariance estimated_error_state_covariance;
  EstimateNominalStateByWheelEncoderMeasurementPrivate(
      nominal_state_, error_state_, error_state_covariance_,
      wheel_encoder_measurement, &reset_error_state,
      &estimated_error_state_covariance, &estimated_nominal_state);

  nominal_state_ = estimated_nominal_state;
  error_state_ = reset_error_state;
  error_state_covariance_ = estimated_error_state_covariance;
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
  const Vec9 predicted_error_state_vector =
      exp_F0dt * previous_error_state.GetErrorStateVector();

  ErrorState predicted_error_state_in_function(current_timestamp,
                                               predicted_error_state_vector);
  *predicted_error_state = predicted_error_state_in_function;

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
  std::cerr << "dt: " << dt << std::endl;

  const double yaw = previous_nominal_state.GetWorldYaw();
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Mat22 R_world_to_imu;
  R_world_to_imu << cy, -sy, sy, cy;

  const auto& am = imu_measurement.GetAccelerationVector();
  const auto gz = imu_measurement.GetYawRate();

  const auto& v = previous_nominal_state.GetWorldVelocity();
  const auto& ba = previous_nominal_state.GetAccXYBias();
  const auto& bg = previous_nominal_state.GetGyroZBias();

  Vec9 derivative_of_previous_nominal_state;
  derivative_of_previous_nominal_state << v, R_world_to_imu * (am - ba),
      gz - bg, 0, O21, 0;

  //   std::cerr << "derivative_of_previous_nominal_state: "
  //             << derivative_of_previous_nominal_state.transpose() <<
  //             std::endl;

  // TODO(@): employ RK4
  // Note: below is the Euler integration.
  const auto& previous_nominal_state_vector =
      previous_nominal_state.GetFullStateVector();
  const Vec9& predicted_nominal_state_vector =
      previous_nominal_state_vector + dt * derivative_of_previous_nominal_state;

  NominalState predicted_nominal_state_in_function(
      current_timestamp, predicted_nominal_state_vector);
  predicted_nominal_state->SetNominalState(predicted_nominal_state_in_function);
  //   std::cerr << "predicted_nominal_state :"
  //             << predicted_nominal_state->GetFullStateVector().transpose()
  //             << std::endl;
}

void WheelImuErrorStateKalmanFilter::
    EstimateNominalStateByWheelEncoderMeasurementPrivate(
        const NominalState& predicted_nominal_state,
        const ErrorState& predicted_error_state,
        const ErrorStateCovariance& predicted_error_state_covariance,
        const WheelEncoderMeasurement& wheel_encoder_measurement,
        ErrorState* reset_error_state,
        ErrorStateCovariance* estimated_error_state_covariance,
        NominalState* estimated_nominal_state) {
  const Mat99& P_predicted =
      predicted_error_state_covariance.GetCovarianceMatrix();
  const Mat22& R = wheel_encoder_measurement_noise_
                       .GetLeftAndRightAngularRateCovarianceMatrix();
  Vec2 z;
  z << wheel_encoder_measurement.GetLeftAngularRate(),
      wheel_encoder_measurement.GetRightAngularRate();

  const double yaw = predicted_nominal_state.GetWorldYaw();
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Mat29 H;
  // clang-format off
  H << 
    O12, 2.0/r_*cy, 2.0/r_*sy, 0, -l_/r_, O12, 0,  //
    O12, 2.0/r_*cy, 2.0/r_*sy, 0,  l_/r_, O12, 0;
  // clang-format on

  const Mat92& K = P_predicted * H.transpose() *
                   (H * P_predicted * H.transpose() + R).inverse();
  //   const Mat99& P_estimated =
  //       (I99 - K * H) * P_predicted * (I99 - K * H).transpose() +
  //       K * R * K.transpose();
  const Mat99& P_estimated = (I99 - K * H) * P_predicted;

  const Vec9 dX_update_vec =
      K * (z - H * predicted_nominal_state.GetFullStateVector());
  const Vec9& dX_estimated =
      predicted_error_state.GetErrorStateVector() + dX_update_vec;

  const Vec9& X_estimated =
      predicted_nominal_state.GetFullStateVector() + dX_estimated;

  *reset_error_state =
      ErrorState(wheel_encoder_measurement.GetTimestamp(), dX_update_vec);
  *estimated_nominal_state =
      NominalState(wheel_encoder_measurement.GetTimestamp(), X_estimated);
  *estimated_error_state_covariance = ErrorStateCovariance(P_estimated);
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

void WheelImuErrorStateKalmanFilter::ShowAll() {
  std::cerr << "Nominal state: "
            << nominal_state_.GetFullStateVector().transpose() << "\n";
  std::cerr << "Error state: " << error_state_.GetErrorStateVector().transpose()
            << "\n";
  std::cerr << "P:\n"
            << error_state_covariance_.GetCovarianceMatrix() << "\n\n"
            << std::endl;
}

}  // namespace error_state_kalman_filter
}  // namespace filter
}  // namespace utility