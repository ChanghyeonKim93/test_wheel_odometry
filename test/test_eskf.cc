#include "wheel_imu_error_state_kalman_filter/types.h"
#include "wheel_imu_error_state_kalman_filter/wheel_imu_error_state_kalman_filter.h"

using namespace utility::filter::error_state_kalman_filter;

int main() {
  WheelImuErrorStateKalmanFilter::Parameters parameters;
  WheelImuErrorStateKalmanFilter eskf(parameters);
  return 0;
}