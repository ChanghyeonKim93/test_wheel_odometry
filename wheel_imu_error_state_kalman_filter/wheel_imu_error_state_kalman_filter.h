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
 public:
  WheelImuErrorStateKalmanFilter() {}

  // Setter
  void SetWheelEncoderNoise();
  void SetImuNoise();
  void SetInitialImuBias();

  void Reset();

  void PredictNominalState();
  void PredictErrorState();
  void EstimateErrorStateByMeasurements();

  void ShowAll();

 public:
  static Mat22 I22;
  static Mat99 I99;

 private:
  NominalState nominal_state_;
  ErrorState error_state_;
  ErrorStateCovariance error_state_covariance_;

  ImuMeasurement imu_measurement_;
  WheelEncoderMeasurement wheel_encoder_measurement_;

  ErrorStateProcessNoise process_noise_of_error_state_;
  WheelEncoderMeasurementNoise wheel_encoder_measurement_noise_;
  ImuMeasurementNoise imu_measurement_noise_;
};

}  // namespace error_state_kalman_filter
}  // namespace filter
}  // namespace utility

#endif