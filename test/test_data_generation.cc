#include <cmath>
#include <fstream>
#include <map>
#include <random>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "matplotlibcpp.h"

#define plt matplotlibcpp
int main() {
  // Set robot parameters
  const double l = 0.5;          // [m]
  const double r = 0.075;        // [m]
  const double time_step{0.01};  // 100 Hz
  const double last_timestamp = 360.0;

  // Generate timestamp list
  std::vector<double> timestamp_list;
  for (double timestamp = 0.0; timestamp <= last_timestamp;
       timestamp += time_step * 0.01)
    timestamp_list.push_back(timestamp);  // 10 kHz data

  // Generate x, y, psi
  std::vector<double> yaw_at_world_list;
  std::vector<double> yaw_rate_at_world_list;
  std::vector<double> yaw_acc_at_world_list;
  std::vector<double> body_vel_list;
  std::vector<double> derivative_body_vel_list;
  for (size_t index = 0; index < timestamp_list.size(); ++index) {
    const double t = timestamp_list[index];

    const double coeff_a = 3.5;
    const double coeff_b = -0.2;
    const double coeff_c = -1.0;
    const double scaler_a = 8.1;
    const double scaler_b = 15.5;
    const double scaler_c = 6.1;

    const double coeff_d = 0.15;
    const double scaler_d = 1.7;

    const double yaw_at_world = coeff_a * std::sin(t / scaler_a) +
                                coeff_b * std::sin(t / scaler_b) +
                                coeff_c * std::sin(t / scaler_c);
    const double yaw_rate_at_world =
        coeff_a / scaler_a * std::cos(t / scaler_a) +
        coeff_b / scaler_b * std::cos(t / scaler_b) +
        coeff_c / scaler_c * std::cos(t / scaler_c);
    const double yaw_acc_at_world =
        -coeff_a / scaler_a / scaler_a * std::sin(t / scaler_a) -
        coeff_b / scaler_b / scaler_b * std::sin(t / scaler_b) -
        coeff_c / scaler_c / scaler_c * std::sin(t / scaler_c);

    const double body_vel = coeff_d * (std::sin(t / scaler_d - M_PI / 2) + 1.2);
    const double derivative_body_vel =
        coeff_d / scaler_d * std::cos(t / scaler_d - M_PI / 2);

    yaw_at_world_list.push_back(yaw_at_world);
    yaw_rate_at_world_list.push_back(yaw_rate_at_world);
    yaw_acc_at_world_list.push_back(yaw_acc_at_world);
    body_vel_list.push_back(body_vel);
    derivative_body_vel_list.push_back(derivative_body_vel);
  }
  // plt::title("body velocity and yaw rate");
  // plt::named_plot("velocity", timestamp_list,
  //                 body_vel_list);  // plot the x,y
  // plt::named_plot("yawing rate", timestamp_list, yaw_rate_at_world_list);
  // plt::grid(true);    // show grid
  // plt::show();        // show figure
  // plt::pause(0.001);  // show figure

  // Generate position, yaw and rate list
  struct State {
    double timestamp;

    double x_at_world;
    double y_at_world;
    double vx_at_world;
    double vy_at_world;
    double ax_at_world;
    double ay_at_world;

    double yaw_at_world;
    double yaw_rate_at_world;
    double yaw_acc_at_world;

    double v_at_body;
    double yaw_rate_at_body;

    double left_angular_velocity;
    double right_angular_velocity;
  };

  std::vector<State> all_state_list;
  State state;
  state.timestamp = 0.0;
  state.vx_at_world = body_vel_list[0] * std::cos(yaw_at_world_list[0]);
  state.vy_at_world = body_vel_list[0] * std::sin(yaw_at_world_list[0]);
  state.ax_at_world =
      derivative_body_vel_list[0] * std::cos(yaw_at_world_list[0]) -
      body_vel_list[0] * std::sin(yaw_at_world_list[0]) *
          yaw_rate_at_world_list[0];
  state.ay_at_world =
      derivative_body_vel_list[0] * std::sin(yaw_at_world_list[0]) +
      body_vel_list[0] * std::cos(yaw_at_world_list[0]) *
          yaw_rate_at_world_list[0];

  state.yaw_at_world = yaw_at_world_list[0];
  state.yaw_rate_at_world = yaw_rate_at_world_list[0];
  state.yaw_acc_at_world = yaw_acc_at_world_list[0];

  state.v_at_body = body_vel_list[0];
  state.yaw_rate_at_body = yaw_rate_at_world_list[0];
  const auto vb = state.v_at_body;
  const auto wb = state.yaw_rate_at_body;
  state.left_angular_velocity = 2.0 / r * vb - l / r * wb;
  state.right_angular_velocity = 2.0 / r * vb + l / r * wb;
  all_state_list.push_back(state);
  for (size_t index = 0; index < timestamp_list.size() - 1; ++index) {
    const double t_next = timestamp_list[index + 1];
    const double t_curr = timestamp_list[index];
    const auto dt = t_next - t_curr;

    const State& current_state = all_state_list[index];
    const auto body_vel = current_state.v_at_body;

    const double yaw_at_world = current_state.yaw_at_world;
    const double cos_yaw = std::cos(yaw_at_world);
    const double sin_yaw = std::sin(yaw_at_world);

    State next_state;
    next_state.timestamp = timestamp_list[index + 1];
    next_state.x_at_world = current_state.x_at_world + dt * body_vel * cos_yaw;
    next_state.y_at_world = current_state.y_at_world + dt * body_vel * sin_yaw;
    next_state.vx_at_world =
        body_vel_list[index + 1] * std::cos(yaw_at_world_list[index + 1]);
    next_state.vy_at_world =
        body_vel_list[index + 1] * std::sin(yaw_at_world_list[index + 1]);
    next_state.ax_at_world = derivative_body_vel_list[index + 1] *
                                 std::cos(yaw_at_world_list[index + 1]) -
                             body_vel_list[index + 1] *
                                 std::sin(yaw_at_world_list[index + 1]) *
                                 yaw_rate_at_world_list[index + 1];
    next_state.ay_at_world = derivative_body_vel_list[index + 1] *
                                 std::sin(yaw_at_world_list[index + 1]) +
                             body_vel_list[index + 1] *
                                 std::cos(yaw_at_world_list[index + 1]) *
                                 yaw_rate_at_world_list[index + 1];

    next_state.yaw_at_world = yaw_at_world_list[index + 1];
    next_state.yaw_rate_at_world = yaw_rate_at_world_list[index + 1];
    next_state.yaw_acc_at_world = yaw_acc_at_world_list[index + 1];

    next_state.v_at_body = body_vel_list[index + 1];
    next_state.yaw_rate_at_body = yaw_rate_at_world_list[index + 1];

    const auto vb = next_state.v_at_body;
    const auto wb = next_state.yaw_rate_at_body;
    next_state.left_angular_velocity = 2.0 / r * vb - l / r * wb;
    next_state.right_angular_velocity = 2.0 / r * vb + l / r * wb;

    all_state_list.push_back(next_state);
  }

  // Text file out
  std::string file_dir = "data.txt";
  std::ofstream output_file(file_dir, std::ios::trunc);
  output_file.precision(4);
  output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
  output_file << "timestamp ";
  output_file << "px ";
  output_file << "py ";
  output_file << "vx ";
  output_file << "vy ";
  output_file << "ax ";
  output_file << "ay ";
  output_file << "yaw ";
  output_file << "yaw_rate ";
  output_file << "yaw_acc ";
  output_file << "vb ";
  output_file << "wb ";
  output_file << "al ";
  output_file << "ar\n";
  if (output_file.is_open()) {
    for (size_t index = 0; index < all_state_list.size(); index += 50) {
      const State& state = all_state_list[index];
      output_file << state.timestamp << " ";
      output_file << state.x_at_world << " ";
      output_file << state.y_at_world << " ";
      output_file << state.vx_at_world << " ";
      output_file << state.vy_at_world << " ";
      output_file << state.ax_at_world << " ";
      output_file << state.ay_at_world << " ";
      output_file << state.yaw_at_world << " ";
      output_file << state.yaw_rate_at_world << " ";
      output_file << state.yaw_acc_at_world << " ";
      output_file << state.v_at_body << " ";
      output_file << state.yaw_rate_at_body << " ";
      output_file << state.left_angular_velocity << " ";
      output_file << state.right_angular_velocity << "\n";
    }
    output_file.close();
  } else {
    std::cerr << "File cannot be open!\n";
    output_file.close();
    return -1;
  }

  // Sample true 100 Hz data
  // std::vector<double> true_timestamp_list;
  // std::vector<double> true_x_list;
  // std::vector<double> true_y_list;
  // std::vector<double> true_vx_list;
  // std::vector<double> true_vy_list;
  // std::vector<double> true_ax_list;
  // std::vector<double> true_ay_list;
  // std::vector<double> true_yaw_list;
  // std::vector<double> true_v_at_body_list;
  // std::vector<double> true_yaw_rate_at_body_list;
  // for (size_t index = 0; index < all_state_list.size(); index += 100) {
  //   const auto& state = all_state_list[index];
  //   true_timestamp_list.push_back(state.timestamp);
  //   true_x_list.push_back(state.x_at_world);
  //   true_y_list.push_back(state.y_at_world);
  //   true_vx_list.push_back(state.vx_at_world);
  //   true_vy_list.push_back(state.vy_at_world);
  //   true_ax_list.push_back(state.ax_at_world);
  //   true_ay_list.push_back(state.ay_at_world);
  //   true_yaw_list.push_back(state.yaw_at_world);
  //   true_v_at_body_list.push_back(state.v_at_body);
  //   true_yaw_rate_at_body_list.push_back(state.yaw_rate_at_body);
  // }

  // plt::named_plot("Robot position", true_x_list,
  //                 true_y_list);  // plot the x,y
  // plt::grid(true);               // show grid
  // plt::show();                   // show figure
  // plt::pause(0.001);             // show figure

  // plt::named_plot("x", true_timestamp_list, true_x_list);
  // plt::named_plot("vx", true_timestamp_list, true_vx_list);
  // plt::named_plot("ax", true_timestamp_list, true_ax_list);
  // plt::grid(true);    // show grid
  // plt::show();        // show figure
  // plt::pause(0.001);  // show figure

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