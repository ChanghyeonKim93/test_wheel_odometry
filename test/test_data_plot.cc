#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "matplotlibcpp.h"

#define plt matplotlibcpp
int main() {
  // Set robot parameters
  const double l = 0.5;    // [m]
  const double r = 0.075;  // [m]

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

  plt::show();  // show figure

  // // Set sensor noise (wheel encoders, IMU angular rates)
  // // Wheel encoders
  // const double pulse_per_revolution = 19.0 * 49.0 * 4.0;
  // const double dt_encoder = 0.01;  // 10 ms
  // const double pulse_error = 1.0;  // 1 pulse error
  // const double noise_encoder =
  //     2.0 * M_PI * pulse_error / pulse_per_revolution / dt_encoder;  // rad/s
  // const double noise_body_velocity_by_encoder =
  //     (noise_encoder * r + noise_encoder * r) / 2.0;
  // const double noise_body_yaw_rate_by_encoder =
  //     (noise_encoder * r + noise_encoder * r) / l;

  // // IMU
  // const double acc_noise_spectral_density = 0.1;    //
  // const double rate_noise_spectral_density = 0.01;  // deg/s/sqrt(Hz)
  // const double noise_acc = rate_noise_spectral_density / 2.0;
  // const double noise_body_yaw_rate_by_imu =
  //     rate_noise_spectral_density / 180.0 * M_PI * std::sqrt(0.005) * 2.0;

  // const double bias_acc_x = -0.001;
  // const double bias_acc_y = 0.003;
  // const double bias_gyro_yaw_rate = 0.001;

  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::normal_distribution<double> dist_wheel_encoder(
  //     0.0, noise_body_yaw_rate_by_encoder);
  // std::normal_distribution<double> dist_acc(0.0, noise_acc);
  // std::normal_distribution<double> dist_yaw_rate(0.0,
  //                                                noise_body_yaw_rate_by_imu);

  // // Generate sensor data list
  // struct EncoderData {
  //   double timestamp{0.0};
  //   double angular_velocity_of_left_wheel{0.0};
  //   double angular_velocity_of_right_wheel{0.0};
  // };
  // struct ImuData {
  //   double timestamp{0.0};
  //   double acc_x{0.0};
  //   double acc_y{0.0};
  //   double yaw_rate{0.0};
  // };

  // Eigen::Matrix2d G;
  // G << r / 2, r / 2, (-r / l), (r / l);
  // const auto iG = G.inverse();

  // std::cout << "r: " << r << std::endl;
  // std::cout << "l: " << l << std::endl;
  // std::cout << "r/l: " << r / l << std::endl;
  // std::cout << G << std::endl;
  // std::cout << iG << std::endl;

  // std::vector<EncoderData> encoder_data_list;  // 50 Hz
  // for (size_t index = 0; index < all_state_list.size(); index += 200) {
  //   const auto& state = all_state_list[index];
  //   Eigen::Vector2d vw;
  //   vw << state.v_at_body, state.yaw_rate_at_body;
  //   const auto alar = iG * vw;

  //   EncoderData encoder_data;
  //   encoder_data.timestamp = state.timestamp;
  //   encoder_data.angular_velocity_of_left_wheel =
  //       alar(0) + dist_wheel_encoder(gen);
  //   encoder_data.angular_velocity_of_right_wheel =
  //       alar(1) + dist_wheel_encoder(gen);

  //   encoder_data_list.push_back(encoder_data);
  // }

  // std::vector<ImuData> imu_data_list;  // 100 Hz
  // for (size_t index = 0; index < all_state_list.size(); index += 100) {
  //   const auto& state = all_state_list[index];
  //   const auto yaw = state.yaw_at_world;
  //   const auto cos_yaw = std::cos(yaw);
  //   const auto sin_yaw = std::sin(yaw);
  //   Eigen::Matrix2d world_to_body_rotation;
  //   world_to_body_rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;

  //   const auto body_to_world_rotation = world_to_body_rotation.transpose();
  //   Eigen::Vector2d axay_at_world;
  //   axay_at_world << state.ax_at_world, state.ay_at_world;
  //   const auto axay_at_body = body_to_world_rotation * axay_at_world;

  //   ImuData imu_data;
  //   imu_data.timestamp = state.timestamp;
  //   imu_data.acc_x = axay_at_body.x() + dist_acc(gen) + bias_acc_x;
  //   imu_data.acc_y = axay_at_body.y() + dist_acc(gen) + bias_acc_y;
  //   imu_data.yaw_rate =
  //       state.yaw_rate_at_body + dist_yaw_rate(gen) + bias_gyro_yaw_rate;

  //   imu_data_list.push_back(imu_data);
  // }

  // std::vector<double> encoder_time_list;
  // std::vector<double> encoder_l_list;
  // std::vector<double> encoder_r_list;
  // for (const auto& encoder_data : encoder_data_list) {
  //   encoder_time_list.push_back(encoder_data.timestamp);
  //   encoder_l_list.push_back(encoder_data.angular_velocity_of_left_wheel);
  //   encoder_r_list.push_back(encoder_data.angular_velocity_of_right_wheel);
  // }
  // plt::title("Encoder data ");
  // plt::named_plot("l", encoder_time_list, encoder_l_list);
  // plt::named_plot("r", encoder_time_list, encoder_r_list);
  // plt::legend();
  // plt::grid(true);    // show grid
  // plt::show();        // show figure
  // plt::pause(0.001);  // show figure

  // std::vector<double> imu_time_list;
  // std::vector<double> imu_ax_list;
  // std::vector<double> imu_ay_list;
  // for (const auto& imu_data : imu_data_list) {
  //   imu_time_list.push_back(imu_data.timestamp);
  //   imu_ax_list.push_back(imu_data.acc_x);
  //   imu_ay_list.push_back(imu_data.acc_y);
  // }

  // plt::title("Imu data ");
  // plt::named_plot("ax", imu_time_list, imu_ax_list);
  // plt::named_plot("ay", imu_time_list, imu_ay_list);
  // plt::legend();
  // plt::grid(true);    // show grid
  // plt::show();        // show figure
  // plt::pause(0.001);  // show figure

  // Implement error state kalman filter

  return 0;
}