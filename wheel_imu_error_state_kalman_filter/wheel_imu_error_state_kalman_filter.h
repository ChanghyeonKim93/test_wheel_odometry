#ifndef UTILITY_FILTER_WHEEL_IMU_ERROR_STATE_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_H_
#define UTILITY_FILTER_WHEEL_IMU_ERROR_STATE_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_H_

#include "eigen3/Eigen/Dense"
#include "wheel_imu_error_state_kalman_filter/types.h"

namespace utility {
namespace filter {
namespace error_state_kalman_filter {

// Note: all states are represented on the IMU frame.
class WheelImuErrorStateKalmanFilter {
 public:
  struct Parameters {
    struct {
      double wheel_radius_in_meter{0.075};  // [m]
      double distance_between_wheels{0.5};  // [m]
    } wheeled_robot_properties;
    struct {
      struct {
        struct {
          double left_angular_rate{0.0};
          double right_angular_rate{0.0};
        } wheel_encoder;
        struct {
          double acc_x{0.0};
          double acc_y{0.0};
          double gyro_z{0.0};
        } imu;
      } measurement;
      struct {
        double position_x{0.001};
        double position_y{0.001};
        double velocity_x{0.002};
        double velocity_y{0.002};
        double yaw{0.001};
        double yaw_rate{0.001};
        double acc_bias_x{1e-5};
        double acc_bias_y{1e-5};
        double gyro_bias_z{1e-5};
      } error_state_process;
    } noise;
    struct {
      struct {
        double acc_x_bias{0.0};   // [m/s^2]
        double acc_y_bias{0.0};   // [m/s^2]
        double gyro_z_bias{0.0};  // [rad/s]
      } imu;
    } initial_bias;
    struct {
      double max_time_difference{0.05};         // [s]
      double max_position_change{0.1};          // [m]
      double max_angle_change{0.2};             // [rad]
      double max_estimated_velocity{4.0};       // [m/s]
      double max_estimated_angular_rate{10.0};  // [rad/s]
    } emergency_reset_rule;
  };

 public:
  explicit WheelImuErrorStateKalmanFilter(const Parameters& parameters);

  void Reset();

  void PredictNominalAndErrorStatesByImuMeasurement(
      const double current_timestamp, const ImuMeasurement& imu_measurement);
  void EstimateNominalStateByWheelEncoderMeasurement(
      const double current_timestamp,
      const WheelEncoderMeasurement& wheel_encoder_measurement);

  const NominalState& GetNominalState() const;
  const ErrorState& GetErrorState() const;
  const ErrorStateCovariance& GetErrorStateCovariance() const;

  void ShowAll();

 private:
  Mat99 CalculateErrorStateTransitionMatrix(
      const double previous_timestamp,
      const NominalState& previous_nominal_state,
      const double current_timestamp, const ImuMeasurement& imu_measurement);

  void PredictErrorStateByImuDeadReckoning(
      const double previous_timestamp, const ErrorState& previous_error_state,
      const NominalState& previous_nominal_state,
      const ErrorStateCovariance& previous_error_state_covariance,
      const double current_timestamp, const ImuMeasurement& imu_measurement,
      ErrorState* predicted_error_state,
      ErrorStateCovariance* predicted_error_state_covariance);
  void PredictNominalStateByImuDeadReckoning(
      const double previous_timestamp,
      const NominalState& previous_nominal_state,
      const double current_timestamp, const ImuMeasurement& imu_measurement,
      NominalState* predicted_nominal_state);

  void EstimateNominalStateByWheelEncoderMeasurementPrivate(
      const NominalState& predicted_nominal_state,
      const ErrorState& predicted_error_state,
      const ErrorStateCovariance& predicted_error_state_covariance,
      const WheelEncoderMeasurement& wheel_encoder_measurement,
      ErrorState* reset_error_state,
      ErrorStateCovariance* estimated_error_state_covariance,
      NominalState* estimated_nominal_state);

 private:
  Parameters parameters_;

  ErrorState error_state_;
  NominalState nominal_state_;
  ErrorStateCovariance error_state_covariance_;

  ErrorStateProcessNoise error_state_process_noise_;
  WheelEncoderMeasurementNoise wheel_encoder_measurement_noise_;
  ImuMeasurementNoise imu_measurement_noise_;

 private:  // robot kinematics
  Mat22 B_;
  Mat22 iB_;
};

}  // namespace error_state_kalman_filter
}  // namespace filter
}  // namespace utility

#endif