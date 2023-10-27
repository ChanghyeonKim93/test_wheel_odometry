#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "matplotlibcpp.h"
#include "wheel_imu_error_state_kalman_filter/types.h"
#include "wheel_imu_error_state_kalman_filter/wheel_imu_error_state_kalman_filter.h"

using namespace utility::filter::error_state_kalman_filter;

#define plt matplotlibcpp
int main() {
  // Set robot parameters
  const double l = 0.5;    // [m]
  const double r = 0.075;  // [m]

  Eigen::Matrix2d G;
  G << r / 2, r / 2, (-r / l), (r / l);
  const auto iG = G.inverse();

  std::vector<double> true_timestamp_list;
  std::vector<double> true_x_list;
  std::vector<double> true_y_list;
  std::vector<double> true_vx_list;
  std::vector<double> true_vy_list;
  std::vector<double> true_ax_list;
  std::vector<double> true_ay_list;
  std::vector<double> true_yaw_list;
  std::vector<double> true_yaw_rate_list;
  std::vector<double> true_yaw_acc_list;
  std::vector<double> true_v_at_body_list;
  std::vector<double> true_yaw_rate_at_body_list;
  std::vector<double> true_left_angular_rate_list;
  std::vector<double> true_right_angular_rate_list;

  // Text file out
  std::string file_dir =
      "/home/kch/Documents/test_wheel_odometry/build/data.txt";
  std::ifstream input_file(file_dir);
  if (!input_file.is_open()) {
    std::cerr << "there is no file." << std::endl;
    return -1;
  }

  // get file 3
  int num_lines = 0;
  char line[300];
  std::string token;
  while (input_file.getline(line, sizeof(line))) {
    if (num_lines == 0) {
      ++num_lines;
      continue;
    }
    double t, x, y, vx, vy, ax, ay, yaw, yaw_rate, yaw_acc, vb, wb, al, ar;
    std::stringstream ss(line);
    std::getline(ss, token, ' ');
    t = std::stof(token);
    std::getline(ss, token, ' ');
    x = std::stof(token);
    std::getline(ss, token, ' ');
    y = std::stof(token);
    std::getline(ss, token, ' ');
    vx = std::stof(token);
    std::getline(ss, token, ' ');
    vy = std::stof(token);
    std::getline(ss, token, ' ');
    ax = std::stof(token);
    std::getline(ss, token, ' ');
    ay = std::stof(token);
    std::getline(ss, token, ' ');
    yaw = std::stof(token);
    std::getline(ss, token, ' ');
    yaw_rate = std::stof(token);
    std::getline(ss, token, ' ');
    yaw_acc = std::stof(token);
    std::getline(ss, token, ' ');
    vb = std::stof(token);
    std::getline(ss, token, ' ');
    wb = std::stof(token);
    std::getline(ss, token, ' ');
    al = std::stof(token);
    std::getline(ss, token, ' ');
    ar = std::stof(token);

    true_timestamp_list.push_back(t);
    true_x_list.push_back(x);
    true_y_list.push_back(y);
    true_vx_list.push_back(vx);
    true_vy_list.push_back(vy);
    true_ax_list.push_back(ax);
    true_ay_list.push_back(ay);
    true_yaw_list.push_back(yaw);
    true_yaw_rate_list.push_back(yaw_rate);
    true_yaw_acc_list.push_back(yaw_acc);
    true_v_at_body_list.push_back(vb);
    true_yaw_rate_at_body_list.push_back(wb);
    true_left_angular_rate_list.push_back(al);
    true_right_angular_rate_list.push_back(ar);

    ++num_lines;
  }
  input_file.close();

  std::cerr << "true_x_list.size(): " << true_x_list.size() << std::endl;

  plt::figure(0);
  plt::title("x-y position");
  plt::named_plot("Robot position", true_x_list,
                  true_y_list);  // plot the x,y
  plt::grid(true);               // show grid

  plt::figure(1);
  plt::title("x, vx, ax");
  plt::named_plot("x", true_timestamp_list, true_x_list);
  plt::named_plot("vx", true_timestamp_list, true_vx_list);
  plt::named_plot("ax", true_timestamp_list, true_ax_list);
  plt::grid(true);  // show grid

  plt::figure(2);
  plt::title("yaw, yaw rate, yaw acc");
  plt::named_plot("y", true_timestamp_list, true_yaw_list);
  plt::named_plot("dy", true_timestamp_list, true_yaw_rate_list);
  plt::named_plot("ddy", true_timestamp_list, true_yaw_acc_list);
  plt::grid(true);  // show grid

  plt::figure(3);
  plt::title("left right angular rate");
  plt::named_plot("left", true_timestamp_list, true_left_angular_rate_list);
  plt::named_plot("right", true_timestamp_list, true_right_angular_rate_list);
  plt::grid(true);  // show grid

  // Set sensor noise (wheel encoders, IMU angular rates)
  // Wheel encoders
  const double pulse_per_revolution = 19.0 * 49.0 * 4.0;
  const double dt_encoder = 0.01;  // 10 ms
  const double pulse_error = 1.0;  // 1 pulse error
  const double noise_encoder =
      2.0 * M_PI * pulse_error / pulse_per_revolution / dt_encoder;  // rad/s
  double noise_body_velocity_by_encoder =
      (noise_encoder * r + noise_encoder * r) / 2.0;
  double noise_body_yaw_rate_by_encoder =
      (noise_encoder * r + noise_encoder * r) / l;
  noise_body_velocity_by_encoder = 0.02;
  noise_body_yaw_rate_by_encoder = 0.01;

  // IMU
  const double acc_noise_spectral_density = 0.1;    //
  const double rate_noise_spectral_density = 0.01;  // deg/s/sqrt(Hz)
  const double noise_acc = rate_noise_spectral_density / 2.0;
  const double noise_body_yaw_rate_by_imu =
      rate_noise_spectral_density / 180.0 * M_PI * std::sqrt(0.005) * 2.0;

  const double bias_acc_x = -0.00;
  const double bias_acc_y = 0.00;
  const double bias_gyro_yaw_rate = 0.000;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist_wheel_encoder_vb(
      0.0, noise_body_velocity_by_encoder);
  std::normal_distribution<double> dist_wheel_encoder_wb(
      0.0, noise_body_yaw_rate_by_encoder);
  std::normal_distribution<double> dist_acc(0.0, noise_acc);
  std::normal_distribution<double> dist_yaw_rate(0.0, 0.001);

  // Generate sensor data list
  struct EncoderData {
    double timestamp{0.0};
    double vb{0.0};
    double wb{0.0};
  };
  struct ImuData {
    double timestamp{0.0};
    double acc_x{0.0};
    double acc_y{0.0};
    double yaw_rate{0.0};
  };

  std::vector<EncoderData> encoder_data_list;  // 50 Hz
  for (size_t index = 0; index < true_x_list.size(); index += 4) {
    const auto timestamp = true_timestamp_list[index];
    const auto true_vb = true_v_at_body_list[index];
    const auto true_yaw_rate = true_yaw_rate_list[index];

    EncoderData encoder_data;
    encoder_data.timestamp = timestamp;
    encoder_data.vb = true_vb + dist_wheel_encoder_vb(gen);
    encoder_data.wb = true_yaw_rate + dist_wheel_encoder_wb(gen);

    encoder_data_list.push_back(encoder_data);
  }

  std::vector<ImuData> imu_data_list;  // 200 Hz
  for (size_t index = 0; index < true_x_list.size(); index += 1) {
    const auto timestamp = true_timestamp_list[index];
    const auto true_ax = true_ax_list[index];
    const auto true_ay = true_ay_list[index];
    const auto true_yaw = true_yaw_list[index];
    const auto true_yaw_rate = true_yaw_rate_list[index];
    const auto cos_yaw = std::cos(true_yaw);
    const auto sin_yaw = std::sin(true_yaw);
    Eigen::Matrix2d world_to_body_rotation;
    world_to_body_rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;

    const auto body_to_world_rotation = world_to_body_rotation.transpose();
    Eigen::Vector2d axay_at_world;
    axay_at_world << true_ax, true_ay;
    const auto axay_at_body = body_to_world_rotation * axay_at_world;

    ImuData imu_data;
    imu_data.timestamp = timestamp;
    imu_data.acc_x = axay_at_body.x() + dist_acc(gen) + bias_acc_x;
    imu_data.acc_y = axay_at_body.y() + dist_acc(gen) + bias_acc_y;
    imu_data.yaw_rate = true_yaw_rate + dist_yaw_rate(gen) + bias_gyro_yaw_rate;

    imu_data_list.push_back(imu_data);
  }

  // ============================================
  // ============================================
  //     Implement error state kalman filter
  // ============================================
  // ============================================
  WheelImuErrorStateKalmanFilter::Parameters parameters;
  WheelImuErrorStateKalmanFilter eskf(parameters);

  size_t imu_index = 0;
  size_t encoder_index = 1;

  std::vector<double> kf_x_list;
  std::vector<double> kf_y_list;
  while (imu_index < imu_data_list.size() &&
         encoder_index < encoder_data_list.size()) {
    const auto& imu_data = imu_data_list[imu_index];
    const double imu_time = imu_data.timestamp;

    eskf.PredictNominalAndErrorStatesByImuMeasurement(
        imu_time, ImuMeasurement(imu_time, imu_data.acc_x, imu_data.acc_y,
                                 imu_data.yaw_rate));

    // const auto& encoder_data = encoder_data_list[encoder_index];
    // const double encoder_time = encoder_data.timestamp;
    // if (imu_time >= encoder_time) {
    //   eskf.EstimateNominalStateByWheelEncoderMeasurement(
    //       encoder_time, WheelEncoderMeasurement(encoder_time,
    //       encoder_data.vb,
    //                                             encoder_data.wb));
    //   ++encoder_index;
    // }

    ++imu_index;

    // eskf.ShowAll();

    std::cout << "Index: " << encoder_index << ", " << imu_index << std::endl;

    kf_x_list.push_back(eskf.GetNominalState().GetWorldPosition().x());
    kf_y_list.push_back(eskf.GetNominalState().GetWorldPosition().y());
  }

  plt::figure(0);
  plt::named_plot("True position", true_x_list, true_y_list);
  plt::named_plot("Dead reckoning", kf_x_list, kf_y_list);
  plt::grid(true);

  plt::show();

  return 0;
}