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
using Mat21 = Eigen::Matrix<double, 2, 1>;
using Mat12 = Eigen::Matrix<double, 1, 2>;
using Mat22 = Eigen::Matrix<double, 2, 2>;
using Mat23 = Eigen::Matrix<double, 2, 3>;
using Mat29 = Eigen::Matrix<double, 2, 9>;
using Mat39 = Eigen::Matrix<double, 3, 9>;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat92 = Eigen::Matrix<double, 9, 2>;
using Mat93 = Eigen::Matrix<double, 9, 3>;
using Mat99 = Eigen::Matrix<double, 9, 9>;

const Mat11 I11 = Mat11::Identity();
const Mat11 O11 = Mat11::Zero();
const Mat22 I22 = Mat22::Identity();
const Mat22 O22 = Mat22::Zero();
const Mat21 I21 = Mat21::Identity();
const Mat21 O21 = Mat21::Zero();
const Mat12 I12 = Mat12::Identity();
const Mat12 O12 = Mat12::Zero();
const Mat99 I99 = Mat99::Identity();

#define GET_BLOCK21(matrix, i, j) ((matrix).block<2, 1>(2 * i, 0))
#define GET_BLOCK22(matrix, i, j) ((matrix).block<2, 2>(2 * i, 2 * j))

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
  ErrorState() : timestamp_(0.0), data_(Vec9::Zero()) {}
  ErrorState(const double timestamp, const Vec9& error_state_in_vector)
      : timestamp_(timestamp), data_(error_state_in_vector) {}
  ErrorState(const ErrorState& dX)
      : timestamp_(dX.timestamp_), data_(dX.data_) {}

  // Getter
  double GetTimestamp() const { return timestamp_; }
  // Vector dX
  Vec9 GetErrorStateVector() const { return data_; }

  // Setter
  void SetZero() { data_.setZero(); }
  void SetErrorState(const ErrorState& rhs) {
    timestamp_ = rhs.timestamp_;
    data_ = rhs.data_;
  }

 private:
  double timestamp_;
  Vec9 data_;
};
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
  NominalState() : timestamp_(0.0), data_(Vec9::Zero()) {}
  NominalState(const double timestamp, const Vec9& state_in_vector)
      : timestamp_(timestamp), data_(state_in_vector) {}
  NominalState(const NominalState& X_nom)
      : timestamp_(X_nom.timestamp_), data_(X_nom.data_) {}

  double GetTimestamp() const { return timestamp_; }
  // Vector X_nominal
  Vec9 GetFullStateVector() const { return data_; }
  Vec2 GetWorldPosition() const { return data_.block<2, 1>(0, 0); }
  Vec2 GetWorldVelocity() const { return data_.block<2, 1>(2, 0); }
  double GetWorldYaw() const { return data_(4); }
  double GetWorldYawRate() const { return data_(5); }
  Vec2 GetAccXYBias() const { return data_.block<2, 1>(6, 0); }
  double GetGyroZBias() const { return data_(8); }

  void SetZeroStateVector() { data_.setZero(); }
  void SetNominalState(const NominalState& rhs) {
    timestamp_ = rhs.timestamp_;
    data_ = rhs.data_;
  }
  void SetWorldPosition(const Vec2& world_position) {
    data_(0) = world_position.x();
    data_(1) = world_position.y();
  }
  void SetWorldVelocity(const Vec2& world_velocity) {
    data_(2) = world_velocity.x();
    data_(3) = world_velocity.y();
  }
  void SetWorldYaw(const double world_yaw) { data_(4) = world_yaw; }
  void SetWorldYawRate(const double world_yaw_rate) {
    data_(5) = world_yaw_rate;
  }
  void SetAccXYBias(const Vec2& acc_xy_bias) {
    data_(6) = acc_xy_bias.x();
    data_(7) = acc_xy_bias.y();
  }
  void SetGyroZBias(const double gyro_z_bias) { data_(8) = gyro_z_bias; }

  void AddErrorState(const ErrorState& error_state) {
    const Vec9& dX = error_state.GetErrorStateVector();
    Vec9& X_nom = data_;
    X_nom += dX;
  }

 private:
  double timestamp_;
  Vec9 data_;
};

class ErrorStateCovariance {
 public:
  ErrorStateCovariance() : data_(Mat99::Identity() * 0.01) {}
  ErrorStateCovariance(const ErrorStateCovariance& rhs) : data_(rhs.data_) {}
  ErrorStateCovariance(const Mat99& P) : data_(P) {}

  // Matrix P
  Mat99 GetCovarianceMatrix() const { return data_; }
  void SetCovarianceMatrix(const ErrorStateCovariance& rhs) {
    data_ = rhs.data_;
  }
  void SetCovarianceMatrix(const Mat99& rhs) { data_ = rhs; }

 private:
  Mat99 data_;  // P marix
};

class ErrorStateProcessNoise {
 public:
  ErrorStateProcessNoise()
      : noise_dp_(Vec2::Zero()),
        noise_dv_(Vec2::Zero()),
        noise_dyaw_(0.0),
        noise_dyaw_rate_(0.0),
        noise_dba_(Vec2::Zero()),
        noise_dbg_(0.0),
        data_(Mat99::Identity()) {}

  // Matrix Q
  Mat99 GetProcessNoiseMatrix() const { return data_; }
  void SetPositionErrorProcessNoise(const Vec2& noise_vec) {
    noise_dp_ = noise_vec;
    data_(0, 0) = noise_dp_.x() * noise_dp_.x();
    data_(1, 1) = noise_dp_.y() * noise_dp_.y();
  }
  void SetVelocityErrorProcessNoise(const Vec2& noise_vec) {
    noise_dv_ = noise_vec;
    data_(2, 2) = noise_dv_.x() * noise_dv_.x();
    data_(3, 3) = noise_dv_.y() * noise_dv_.y();
  }
  void SetYawErrorProcessNoise(const double noise) {
    noise_dyaw_ = noise;
    data_(4, 4) = noise_dyaw_ * noise_dyaw_;
  }
  void SetYawRateErrorProcessNoise(const double noise) {
    noise_dyaw_rate_ = noise;
    data_(5, 5) = noise_dyaw_rate_ * noise_dyaw_rate_;
  }
  void SetAccXYBiasErrorProcessNoise(const Vec2& noise_vec) {
    noise_dba_ = noise_vec;
    data_(6, 6) = noise_dba_.x() * noise_dba_.x();
    data_(7, 7) = noise_dba_.y() * noise_dba_.y();
  }
  void SetGyroZBiasErrorProcessNoise(const double noise) {
    noise_dbg_ = noise;
    data_(8, 8) = noise_dbg_ * noise_dbg_;
  }

 private:
  Vec2 noise_dp_;
  Vec2 noise_dv_;
  double noise_dyaw_;
  double noise_dyaw_rate_;
  Vec2 noise_dba_;
  double noise_dbg_;

  Mat99 data_;
};

class ImuMeasurement {
 public:
  ImuMeasurement() : timestamp_(0.0), data_(Vec3::Zero()) {}
  ImuMeasurement(const double timestamp, const double acc_x, const double acc_y,
                 const double yaw_rate)
      : timestamp_(timestamp), data_(acc_x, acc_y, yaw_rate) {}
  ImuMeasurement(const ImuMeasurement& rhs)
      : timestamp_(rhs.timestamp_), data_(rhs.data_) {}

  // Getter
  double GetTimestamp() const { return timestamp_; }
  double GetAccerelationX() const { return data_.x(); }
  double GetAccerelationY() const { return data_.y(); }
  Vec2 GetAccelerationVector() const { return data_.block<2, 1>(0, 0); }
  double GetYawRate() const { return data_.z(); }

 private:
  double timestamp_;
  Vec3 data_;  // acc_x, acc_y, yaw_rate
};

class WheelEncoderMeasurement {
 public:
  WheelEncoderMeasurement()
      : timestamp_(0.0), left_angular_rate_(0.0), right_angular_rate_(0.0) {}
  WheelEncoderMeasurement(const double timestamp,
                          const double left_angular_rate,
                          const double right_angular_rate)
      : timestamp_(timestamp),
        left_angular_rate_(left_angular_rate),
        right_angular_rate_(right_angular_rate) {}
  WheelEncoderMeasurement(const WheelEncoderMeasurement& rhs)
      : timestamp_(rhs.timestamp_),
        left_angular_rate_(rhs.left_angular_rate_),
        right_angular_rate_(rhs.right_angular_rate_) {}

  // Getter
  double GetTimestamp() const { return timestamp_; }
  double GetLeftAngularRate() const { return left_angular_rate_; }
  double GetRightAngularRate() const { return right_angular_rate_; }

 private:
  double timestamp_;
  double left_angular_rate_;
  double right_angular_rate_;
};

class ImuMeasurementNoise {
 public:
  ImuMeasurementNoise()
      : acc_x_noise_(0.0),
        acc_y_noise_(0.0),
        gyro_z_noise_(0.0),
        noise_covariance_matrix_for_acc_xy_(Mat22::Zero()),
        noise_covariance_matrix_for_gyro_z_(0.0) {}

  void SetAccXYNoise(const double acc_x_noise, const double acc_y_noise) {
    acc_x_noise_ = acc_x_noise;
    acc_y_noise_ = acc_y_noise;
    noise_covariance_matrix_for_acc_xy_(0, 0) = acc_x_noise_ * acc_x_noise_;
    noise_covariance_matrix_for_acc_xy_(1, 1) = acc_y_noise_ * acc_y_noise_;
  }
  void SetGyroZNoise(const double gyro_z_noise) {
    gyro_z_noise_ = gyro_z_noise;
    noise_covariance_matrix_for_gyro_z_ = gyro_z_noise_ * gyro_z_noise_;
  }
  // ### Matrix R (2x2 acc xy part)
  Mat22 GetAccXYNoiseCovarianceMatrix() const {
    return noise_covariance_matrix_for_acc_xy_;
  }
  // ### Matrix R(1x1 gyro z part)
  double GetGyroZNoiseCovariance() const {
    return noise_covariance_matrix_for_gyro_z_;
  }

 private:
  double acc_x_noise_;
  double acc_y_noise_;
  double gyro_z_noise_;
  Mat22 noise_covariance_matrix_for_acc_xy_;
  double noise_covariance_matrix_for_gyro_z_;
};

class WheelEncoderMeasurementNoise {
 public:
  WheelEncoderMeasurementNoise()
      : left_angular_rate_noise_(0.0),
        right_angular_rate_noise_(0.0),
        noise_covariance_matrix_(Mat22::Zero()) {}
  void SetLeftAndRightAngularRateNoises(const double left_angular_rate_noise,
                                        const double right_angular_rate_noise) {
    left_angular_rate_noise_ = left_angular_rate_noise;
    right_angular_rate_noise_ = right_angular_rate_noise;
    noise_covariance_matrix_.setZero();
    noise_covariance_matrix_(0, 0) =
        left_angular_rate_noise_ * left_angular_rate_noise_;
    noise_covariance_matrix_(1, 1) =
        right_angular_rate_noise_ * right_angular_rate_noise_;
  }

  // ### Matrix diag([n_al^2, n_ar^2])
  Mat22 GetLeftAndRightAngularRateCovarianceMatrix() const {
    return noise_covariance_matrix_;
  }

 private:
  double left_angular_rate_noise_;
  double right_angular_rate_noise_;
  Mat22 noise_covariance_matrix_;
};

}  // namespace error_state_kalman_filter
}  // namespace filter
}  // namespace utility

#endif