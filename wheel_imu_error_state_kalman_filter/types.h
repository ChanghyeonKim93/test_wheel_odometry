#ifndef UTILITY_FILTER_WHEEL_IMU_ERROR_STATE_KALMAN_FILTER_TYPES_H_
#define UTILITY_FILTER_WHEEL_IMU_ERROR_STATE_KALMAN_FILTER_TYPES_H_

#include "eigen3/Eigen/Dense"

namespace utility {
namespace filter {
namespace error_state_kalman_filter {

using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat11 = Eigen::Matrix<double, 1, 1>;
using Mat22 = Eigen::Matrix<double, 2, 2>;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat99 = Eigen::Matrix<double, 9, 9>;

const Mat22 I22 = Mat22::Identity();
const Mat99 I99 = Mat99::Identity();

#define GET_BLOCK21(matrix, i, j) ((matrix).block<2, 1>(2 * i, 0))
#define GET_BLOCK22(matrix, i, j) ((matrix).block<2, 2>(2 * i, 2 * j))

class NominalState;
class ErrorState;
class ErrorStateCovariance;
class ErrorStateProcessNoise;
class WheelEncoderMeasurement;
class ImuMeasurement;
class WheelEncoderMeasurementNoise;
class ImuMeasurementNoise;

class NominalState {
  /*
   p:        data_(0:1)
   v:        data_(2:3)
   yaw:      data_(4:4)
   yaw_rate: data_(5:5)
   ba:       data_(6:7)
   bg:       data_(8:8)
   */
 public:
  NominalState() { data_ = Vec9::Zero(); }
  NominalState(const double timestamp, const Vec9& state_in_vector) {
    timestamp_ = timestamp;
    data_ = state_in_vector;
  }
  NominalState(const NominalState& X_nom) {
    timestamp_ = X_nom.timestamp_;
    data_ = X_nom.data_;
  }

  // Getter
  double GetTimestamp() const { return timestamp_; }
  Vec9 GetStateVector() const { return data_; }
  Vec2 GetPartOfWorldPosition() const { return data_.block<2, 1>(0, 0); }
  Vec2 GetPartOfWorldVelocity() const { return data_.block<2, 1>(2, 0); }
  Vec2 GetPartOfWorldYaw() const { return data_.block<1, 1>(4, 0); }
  Vec2 GetPartOfWorldYawRate() const { return data_.block<1, 1>(5, 0); }
  Vec2 GetPartOfAccXYBias() const { return data_.block<2, 1>(6, 0); }
  Vec2 GetPartOfGyroZBias() const { return data_.block<1, 1>(8, 0); }

  // Setter
  void SetZeroStateVector() { data_.setZero(); }
  void SetStateVector(const NominalState& rhs) { data_ = rhs.data_; }

  // Add
  void AddErrorState(const ErrorState& error_state) {
    const Vec9& dX = error_state.GetErrorStateVector();
    Vec9& X_nom = data_;
    X_nom += dX;
  }

 private:
  double timestamp_;
  Vec9 data_;
};

class ErrorState {
  /*
   dp:        data_(0:1)
   dv:        data_(2:3)
   dyaw:      data_(4:4)
   dyaw_rate: data_(5:5)
   dba:       data_(6:7)
   dbg:       data_(8:8)
   */
 public:
 public:
  ErrorState() { data_ = Vec9::Zero(); }
  ErrorState(const double timestamp, const Vec9& error_state_in_vector) {
    timestamp_ = timestamp;
    data_ = error_state_in_vector;
  }
  ErrorState(const ErrorState& dX) {
    timestamp_ = dX.timestamp_;
    data_ = dX.data_;
  }

  // Getter
  double GetTimestamp() const { return timestamp_; }
  Vec9 GetErrorStateVector() const { return data_; }

  // Setter
  void SetZeroErrorStateVector() { data_.setZero(); }
  void SetErrorStateVector(const ErrorState& rhs) { data_ = rhs.data_; }

 private:
  double timestamp_;
  Vec9 data_;
};

class ErrorStateCovariance {
 public:
  ErrorStateCovariance() { data_ = Mat99::Identity() * 0.01; }
  ErrorStateCovariance(const ErrorStateCovariance& rhs) { data_ = rhs.data_; }
  ErrorStateCovariance(const Mat99& P) { data_ = P; }

  // Getter
  Mat99 GetCovarianceMatrix() const { return data_; }

  // Setter
  void SetCovarianceMatrix(const ErrorStateCovariance& rhs) {
    data_ = rhs.data_;
  }
  void SetCovarianceMatrix(const Mat99& rhs) { data_ = rhs; }

 private:
  Mat99 data_;  // P marix
};

class ErrorStateProcessNoise {
 public:
  ErrorStateProcessNoise() {}
  ~ErrorStateProcessNoise() {}

  // Getter
  Mat99 GetProcessNoiseMatrix() const { return data_; }

  // Setter
  void SetErrorStateProcessNoiseMatrix(const Mat99& rhs) { data_ = rhs; }

 private:
  double noise_dp_;
  double noise_dv_;
  double noise_dyaw_;
  double noise_dyaw_;
  Mat99 data_;
};

class WheelEncoderMeasurement {
 public:
  WheelEncoderMeasurement() : timestamp_(0.0), data_(Vec2::Zero()) {}
  WheelEncoderMeasurement(const double timestamp,
                          const double left_angular_rate,
                          const double right_angular_rate) {
    timestamp_ = timestamp;
    data_ << left_angular_rate, right_angular_rate;
  }
  WheelEncoderMeasurement(const WheelEncoderMeasurement& rhs) {
    timestamp_ = rhs.timestamp_;
    data_ = rhs.data_;
  }

  // Getter
  double GetLeftAngularRate() const { return data_.x(); }
  double GetRightAngularRate() const { return data_.y(); }
  Vec2 GetLeftAndRightAngularRateVector() const { return data_; }

 private:
  double timestamp_;
  Vec2 data_;  // left/right angular rates
};

class ImuMeasurement {
 public:
  ImuMeasurement() : timestamp_(0.0), data_(Vec3::Zero()) {}
  ImuMeasurement(const double timestamp, const double acc_x, const double acc_y,
                 const double yaw_rate) {
    timestamp_ = timestamp;
    data_ << acc_x, acc_y, yaw_rate;
  }
  ImuMeasurement(const ImuMeasurement& rhs) {
    timestamp_ = rhs.timestamp_;
    data_ = rhs.data_;
  }

  // Getter
  double GetAccerelationX() const { return data_.x(); }
  double GetAccerelationY() const { return data_.y(); }
  Vec2 GetAccelerationVector() const { return data_.block<2, 1>(0, 0); }
  double GetYawRate() const { return data_.z(); }

 private:
  double timestamp_;
  Vec3 data_;  // acc_x, acc_y, yaw_rate
};

class ImuMeasurementNoise {
 public:
  ImuMeasurementNoise() : data_(Vec3::Zero()) {}
  ImuMeasurementNoise(const double acc_x_noise, const double acc_y_noise,
                      const double yaw_rate_noise) {
    data_ << acc_x_noise, acc_y_noise, yaw_rate_noise;
    noise_covariance_matrix_for_acc_xy_.setZero();
    noise_covariance_matrix_for_acc_xy_(0, 0) = acc_x_noise * acc_x_noise;
    noise_covariance_matrix_for_acc_xy_(1, 1) = acc_y_noise * acc_y_noise;
    noise_covariance_matrix_for_gyro_z_ = yaw_rate_noise * yaw_rate_noise;
  }

  // Getter
  Mat22 GetAccelerometerXYNoiseCovarianceMatrix() const {
    return noise_covariance_matrix_for_acc_xy_;
  }
  double GetGyroscopeZNoiseCovariance() const {
    return noise_covariance_matrix_for_gyro_z_;
  }

 private:
  Vec3 data_;
  Mat22 noise_covariance_matrix_for_acc_xy_;
  double noise_covariance_matrix_for_gyro_z_;
};

class WheelEncoderMeasurementNoise {
 public:
  WheelEncoderMeasurementNoise() : data_(Vec2::Zero()) {}
  WheelEncoderMeasurementNoise(const double left_wheel_angular_rate_noise,
                               const double right_wheel_angular_rate_noise) {
    data_ << left_wheel_angular_rate_noise, right_wheel_angular_rate_noise;
    noise_covariance_matrix_for_wheel_angular_rate_left_right_ = Mat22::Zero();
    noise_covariance_matrix_for_wheel_angular_rate_left_right_(0, 0) =
        left_wheel_angular_rate_noise * left_wheel_angular_rate_noise;
    noise_covariance_matrix_for_wheel_angular_rate_left_right_(1, 1) =
        right_wheel_angular_rate_noise * right_wheel_angular_rate_noise;
  }

  // Getter
  Mat22 GetLeftRightWheelAngularRateCovarianceMatrix() const {
    return noise_covariance_matrix_for_wheel_angular_rate_left_right_;
  }

 private:
  Vec2 data_;
  Mat22 noise_covariance_matrix_for_wheel_angular_rate_left_right_;
};

}  // namespace error_state_kalman_filter
}  // namespace filter
}  // namespace utility

#endif