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
      double wheel_radius_in_meter{0.0};
      double distance_between_wheels{0.0};
    } wheeled_robot_properties;
    struct {
      struct {
        double left_wheel_angular_rate{0.0};
        double right_wheel_angular_rate{0.0};
      } wheel_encoder;
      struct {
        double acc_x{0.0};
        double acc_y{0.0};
        double gyro_z{0.0};
      } imu;
    } noise;

    struct {
      struct {
        double acc_x_bias{0.0};
        double acc_y_bias{0.0};
        double gyro_z_bias{0.0};
      } imu;
    } bias;

    struct {
      double max_time_difference;
      double max_position_change;
      double max_angle_change;
      double max_estimated_velocity;
      double max_estimated_angular_rate;
    } emergency_reset_rule;
  };

 public:
  explicit WheelImuErrorStateKalmanFilter(const Parameters& parameters);

  void Reset();

  void PredictNominalStateByImuDeadReckoning(const double timestamp,
                                             const Vec2& acc_xy,
                                             const double gyro_z);
  void PredictErrorStateByImuDeadReckoning(const double timestamp,
                                           const Vec2& acc_xy,
                                           const double gyro_z);
  void EstimateErrorStateByWheelEncoderMeasurements();

  const NominalState& GetNominalState() const;
  const ErrorState& GetErrorState() const;
  const ErrorStateCovariance& GetErrorStateCovariance() const;

  void ShowAll();

 private:
  void SetWheelEncoderNoise();
  void SetImuNoise();
  void SetInitialImuBias();

 private:
  Parameters parameters_;

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