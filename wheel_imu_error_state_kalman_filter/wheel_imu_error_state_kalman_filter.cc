#include "wheel_imu_error_state_kalman_filter/wheel_imu_error_state_kalman_filter.h"

namespace utility {
namespace filter {
namespace error_state_kalman_filter {

explicit WheelImuErrorStateKalmanFilter::WheelImuErrorStateKalmanFilter(
    const Parameters& parameters)
    : parameters_(parameters) {}

}  // namespace error_state_kalman_filter
}  // namespace filter
}  // namespace utility