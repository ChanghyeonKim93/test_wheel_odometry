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

  // Set sensor noise (wheel encoders, IMU angular rates)
  // Wheel encoders
  const double pulse_per_revolution = 4096.0;
  const double dt_encoder = 0.02;  // 50 Hz
  const double pulse_error = 1.0;  // 1 pulse error
  const double noise_encoder =
      2.0 * M_PI * pulse_error / pulse_per_revolution / dt_encoder;  // rad/s
  std::cout << "noise encoder: " << noise_encoder << std::endl;
  const double noise_body_velocity_by_encoder =
      (noise_encoder * r + noise_encoder * r) / 2.0;
  const double noise_body_yaw_rate_by_encoder =
      (noise_encoder * r + noise_encoder * r) / l;

  // IMU
  const double acc_noise_spectral_density = 0.1;    //
  const double rate_noise_spectral_density = 0.01;  // deg/s/sqrt(Hz)
  const double noise_acc = rate_noise_spectral_density / 2.0;
  const double noise_body_yaw_rate_by_imu =
      rate_noise_spectral_density / 180.0 * M_PI * std::sqrt(0.005) * 2.0;

  const double bias_acc_x = -0.001;
  const double bias_acc_y = 0.003;
  const double bias_gyro_yaw_rate = 0.001;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist_wheel_encoder(
      0.0, noise_body_yaw_rate_by_encoder);
  std::normal_distribution<double> dist_acc(0.0, noise_acc);
  std::normal_distribution<double> dist_vb(0.0, noise_body_velocity_by_encoder);
  std::normal_distribution<double> dist_wb(0.0, noise_body_yaw_rate_by_encoder);
  std::normal_distribution<double> dist_wi(0.0, noise_body_yaw_rate_by_imu);

  const int sampling_step = 4;  // 50 Hz
  struct State {
    double timestamp;
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double yaw;
    double yaw_rate;

    double vb;
    double wb;
    double wi;
  };

  std::vector<State> odometry_state_list;
  std::vector<double> odometry_time_list;
  std::vector<double> odometry_x_list;
  std::vector<double> odometry_y_list;
  std::vector<double> odometry_vx_list;
  std::vector<double> odometry_vy_list;
  std::vector<double> odometry_yaw_list;
  for (size_t index = 0; index < true_timestamp_list.size();
       index += sampling_step) {
    const auto time = true_timestamp_list[index];
    const auto vb = true_v_at_body_list[index];
    const auto wb = true_yaw_rate_at_body_list[index];
    const auto wi = true_yaw_rate_at_body_list[index];

    State state;
    state.timestamp = time;
    state.vb = vb + dist_vb(gen);
    state.wb = wb + dist_wb(gen);
    state.wi = wi + dist_wi(gen);
    odometry_state_list.push_back(state);
  }

  odometry_state_list.front().pos.x() = 0.0;
  odometry_state_list.front().pos.y() = 0.0;
  odometry_state_list.front().yaw = 0.0;
  odometry_time_list.push_back(odometry_state_list.front().timestamp);
  odometry_x_list.push_back(odometry_state_list.front().pos.x());
  odometry_y_list.push_back(odometry_state_list.front().pos.y());
  odometry_vx_list.push_back(odometry_state_list.front().vel.x());
  odometry_vy_list.push_back(odometry_state_list.front().vel.y());
  odometry_yaw_list.push_back(odometry_state_list.front().yaw);
  for (size_t index = 1; index < odometry_state_list.size(); ++index) {
    auto& prev_state = odometry_state_list[index - 1];
    auto& curr_state = odometry_state_list[index];
    const double cos_yaw = std::cos(prev_state.yaw);
    const double sin_yaw = std::sin(prev_state.yaw);

    const double dt = curr_state.timestamp - prev_state.timestamp;
    Eigen::Vector2d v_world;
    v_world.x() = prev_state.vb * cos_yaw;
    v_world.y() = prev_state.vb * sin_yaw;
    curr_state.pos = prev_state.pos + dt * v_world;
    curr_state.vel = v_world;
    curr_state.yaw = prev_state.yaw + dt * prev_state.wb;

    odometry_time_list.push_back(curr_state.timestamp);
    odometry_x_list.push_back(curr_state.pos.x());
    odometry_y_list.push_back(curr_state.pos.y());
    odometry_vx_list.push_back(curr_state.vel.x());
    odometry_vy_list.push_back(curr_state.vel.y());
    odometry_yaw_list.push_back(curr_state.yaw);
  }

  plt::figure(4);
  plt::title("x-y position (comparison)");
  plt::named_plot("True", true_x_list,
                  true_y_list);  // plot the x,y
  plt::named_plot("Odometry", odometry_x_list,
                  odometry_y_list);  // plot the x,y
  plt::grid(true);                   // show grid

  plt::figure(5);
  plt::title("y (comparison)");
  plt::named_plot("True", true_timestamp_list,
                  true_y_list);  // plot the x,y
  plt::named_plot("Odometry", odometry_time_list,
                  odometry_y_list);  // plot the x,y
  plt::grid(true);                   // show grid

  // Simple KF
  const double noise_pos_process = 0.01;
  const double noise_vel_process = 0.01;
  const double noise_yaw_process = 0.03;
  const double noise_yaw_rate_process = 0.01;
  Eigen::Matrix<double, 2, 2> I22 = Eigen::Matrix<double, 2, 2>::Identity();
  Eigen::Matrix<double, 6, 6> I66 = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 1, 2> O12 = Eigen::Matrix<double, 1, 2>::Zero();
  Eigen::Matrix<double, 2, 1> O21 = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, 2, 2> O22 = Eigen::Matrix<double, 2, 2>::Zero();
  Eigen::Matrix<double, 3, 6> H;
  Eigen::Matrix<double, 6, 6> F;
  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 6, 6> P;
  Eigen::Matrix<double, 3, 3> R;
  Eigen::Matrix<double, 6, 3> K;
  R(0, 0) = noise_body_velocity_by_encoder * noise_body_velocity_by_encoder;
  R(1, 1) = R(0, 0) * 0.01;
  R(2, 2) = noise_body_yaw_rate_by_encoder * noise_body_yaw_rate_by_encoder;
  Q.setIdentity();
  Q(0, 0) = noise_pos_process * noise_pos_process;
  Q(1, 1) = noise_pos_process * noise_pos_process;
  Q(2, 2) = noise_vel_process * noise_vel_process;
  Q(3, 3) = noise_vel_process * noise_vel_process;
  Q(4, 4) = noise_yaw_process * noise_yaw_process;
  Q(5, 5) = noise_yaw_rate_process * noise_yaw_rate_process;
  P.setIdentity();
  P *= 10.1;

  std::vector<State> kf_state_list;
  std::vector<double> kf_time_list;
  std::vector<double> kf_x_list;
  std::vector<double> kf_y_list;
  std::vector<double> kf_vx_list;
  std::vector<double> kf_vy_list;
  std::vector<double> kf_yaw_list;
  for (size_t index = 0; index < true_timestamp_list.size();
       index += sampling_step) {
    const auto time = true_timestamp_list[index];
    const auto vb = true_v_at_body_list[index];
    const auto wb = true_yaw_rate_at_body_list[index];
    const auto wi = true_yaw_rate_at_body_list[index];

    State state;
    state.timestamp = time;
    state.vb = vb + dist_vb(gen);
    state.wb = wb + dist_wb(gen);
    state.wi = wi + dist_wi(gen);
    kf_state_list.push_back(state);
  }

  kf_state_list.front().pos.x() = 0.0;
  kf_state_list.front().pos.y() = 0.0;
  kf_state_list.front().vel.x() = true_vx_list.front();
  kf_state_list.front().vel.y() = true_vy_list.front();
  kf_state_list.front().yaw = 0.0;
  kf_state_list.front().yaw_rate = true_yaw_rate_at_body_list.front();
  kf_time_list.push_back(kf_state_list.front().timestamp);
  kf_x_list.push_back(kf_state_list.front().pos.x());
  kf_y_list.push_back(kf_state_list.front().pos.y());
  kf_vx_list.push_back(kf_state_list.front().vel.x());
  kf_vy_list.push_back(kf_state_list.front().vel.y());
  kf_yaw_list.push_back(kf_state_list.front().yaw);
  for (size_t index = 1; index < kf_state_list.size(); ++index) {
    auto& prev_state = kf_state_list[index - 1];
    auto& curr_state = kf_state_list[index];

    const double dt = curr_state.timestamp - prev_state.timestamp;

    // predict
    Eigen::Vector2d v_world = prev_state.vel;
    F << I22, dt * I22, O21, O21, O22, I22, O21, O21, O12, O12, 1, 0, O12, O12,
        0, 1;
    curr_state.pos = prev_state.pos + dt * v_world;
    curr_state.vel = prev_state.vel;
    curr_state.yaw = prev_state.yaw + dt * prev_state.wi;
    curr_state.yaw_rate = prev_state.wi;
    P = F * P * F.transpose() + Q;

    const double cos_yaw_predicted = std::cos(curr_state.yaw);
    const double sin_yaw_predicted = std::sin(curr_state.yaw);
    Eigen::Matrix2d rot_predicted;
    rot_predicted << cos_yaw_predicted, -sin_yaw_predicted, sin_yaw_predicted,
        cos_yaw_predicted;

    // estimate
    H << O22, rot_predicted.transpose(), O21, O21, O12, O12, 0, 1;
    Eigen::Vector3d z;
    z << curr_state.vb, 0.0, curr_state.wb;
    Eigen::Vector3d z_hat;
    Eigen::Matrix<double, 6, 1> curr_state_predicted;
    curr_state_predicted << curr_state.pos, curr_state.vel, curr_state.yaw,
        curr_state.yaw_rate;
    z_hat = H * curr_state_predicted;

    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    P = (I66 - K * H) * P * (I66 - K * H).transpose() + K * R * K.transpose();
    const auto curr_state_estimated = curr_state_predicted + K * (z - z_hat);
    curr_state.pos = curr_state_estimated.block<2, 1>(0, 0);
    curr_state.vel = curr_state_estimated.block<2, 1>(2, 0);
    curr_state.yaw = curr_state_estimated(4);
    curr_state.yaw_rate = curr_state_estimated(5);

    kf_time_list.push_back(curr_state.timestamp);
    kf_x_list.push_back(curr_state.pos.x());
    kf_y_list.push_back(curr_state.pos.y());
    kf_vx_list.push_back(curr_state.vel.x());
    kf_vy_list.push_back(curr_state.vel.y());
    kf_yaw_list.push_back(curr_state.yaw);
  }

  plt::figure(6);
  plt::title("x-y position (comparison)");
  plt::named_plot("True", true_x_list, true_y_list, "k-");
  plt::named_plot("Odometry", odometry_x_list, odometry_y_list, "b-");
  plt::named_plot("KF", kf_x_list, kf_y_list, "r--");
  plt::grid(true);  // show grid

  plt::figure(7);
  plt::title("yaw (comparison)");
  plt::named_plot("True", true_timestamp_list, true_yaw_list, "k-");
  plt::named_plot("Odometry", odometry_time_list, odometry_yaw_list, "b-");
  plt::named_plot("KF", kf_time_list, kf_yaw_list, "r--");
  plt::grid(true);  // show grid

  plt::figure(8);
  plt::title("vel x (comparison)");
  plt::named_plot("True", true_timestamp_list, true_vx_list, "k-");
  plt::named_plot("Odometry", odometry_time_list, odometry_vx_list, "b-");
  plt::named_plot("KF", kf_time_list, kf_vx_list, "r--");
  plt::grid(true);  // show grid

  plt::figure(9);
  plt::title("vel y (comparison)");
  plt::named_plot("True", true_timestamp_list, true_vy_list, "k-");
  plt::named_plot("Odometry", odometry_time_list, odometry_vy_list, "b-");
  plt::named_plot("KF", kf_time_list, kf_vy_list, "r--");
  plt::grid(true);  // show grid
  plt::show();

  // // Set sensor noise (wheel encoders, IMU angular rates)
  // // Wheel encoders
  // const double pulse_per_revolution = 19.0 * 49.0 * 4.0;
  // const double dt_encoder = 0.01;  // 10 ms
  // const double pulse_error = 1.0;  // 1 pulse error
  // const double noise_encoder =
  //     2.0 * M_PI * pulse_error / pulse_per_revolution / dt_encoder;  //
  //     rad/s
  // const double noise_body_velocity_by_encoder =
  //     (noise_encoder * r + noise_encoder * r) / 2.0;
  // const double noise_body_yaw_rate_by_encoder =
  //     (noise_encoder * r + noise_encoder * r) / l;

  // // IMU
  // const double acc_noise_spectral_density = 0.1;    //
  // const double rate_noise_spectral_density = 0.01;  // deg/s/sqrt(Hz)
  // const double noise_acc = rate_noise_spectral_density / 2.0;
  // const double noise_body_yaw_rate_by_imu =
  //     rate_noise_spectral_density / 180.0 * M_PI * std::sqrt(0.005)
  //     * 2.0;

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

  //   const auto body_to_world_rotation =
  //   world_to_body_rotation.transpose(); Eigen::Vector2d axay_at_world;
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