#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{
using Axes = std::array<double, 3>;
using Imu = sensor_msgs::msg::Imu;
constexpr double kGravity = 9.80665;

// Keep the Livox settings and update math in sync with gyro_bias.py.
class BiasTracker
{
public:
  bool update(const Axes & gyro, const Axes & accel)
  {
    const std::array<double, 6> sample{gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]};
    for (std::size_t i = 0; i < sample.size(); ++i) {
      sum_[i] += sample[i];
      sum_squares_[i] += sample[i] * sample[i];
    }
    if (++count_ < kWindow) {
      return false;
    }

    std::array<double, 6> mean, sd;
    for (std::size_t i = 0; i < sample.size(); ++i) {
      mean[i] = sum_[i] / count_;
      sd[i] = std::sqrt(std::max(sum_squares_[i] / count_ - mean[i] * mean[i], 0.0));
    }
    count_ = 0;
    sum_.fill(0.0);
    sum_squares_.fill(0.0);

    const bool quiet =
      std::max({sd[0], sd[1], sd[2]}) < kQuietGyroSd &&
      std::max({sd[3], sd[4], sd[5]}) < kQuietAccelSd &&
      std::max({std::abs(mean[0]), std::abs(mean[1]), std::abs(mean[2])}) < kMaxBias;
    if (!quiet) {
      quiet_run_ = 0;
      return false;
    }
    if (++quiet_run_ < kQuietWindows) {
      return false;
    }

    const double gain = updates_ == 0 ? 1.0 : kBiasGain;
    for (std::size_t i = 0; i < bias_.size(); ++i) {
      bias_[i] += gain * (mean[i] - bias_[i]);
    }
    ++updates_;
    return true;
  }

  const Axes & bias() const {return bias_;}
  std::size_t updates() const {return updates_;}

private:
  static constexpr int kWindow = 200;
  static constexpr double kQuietGyroSd = 0.004;
  static constexpr double kQuietAccelSd = 0.1;
  static constexpr int kQuietWindows = 2;
  static constexpr double kMaxBias = 0.02;
  static constexpr double kBiasGain = 0.2;

  Axes bias_{};
  std::size_t updates_ = 0;
  int quiet_run_ = 0;
  int count_ = 0;
  std::array<double, 6> sum_{}, sum_squares_{};
};

class LivoxImuRelay : public rclcpp::Node
{
public:
  LivoxImuRelay()
  : Node("livox_imu_relay_cpp")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/livox/imu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "imu/data_raw");
    output_.header.frame_id = declare_parameter<std::string>("frame_id", "livox_frame");
    restamp_ = declare_parameter("restamp", true);
    const auto gyro_var = declare_parameter("gyro_var", 1e-4);
    const auto accel_var = declare_parameter("accel_var", 1e-2);
    track_bias_ = declare_parameter("track_bias", true);

    output_.orientation_covariance[0] = -1.0;
    output_.angular_velocity_covariance = {gyro_var, 0, 0, 0, gyro_var, 0, 0, 0, gyro_var};
    output_.linear_acceleration_covariance = {accel_var, 0, 0, 0, accel_var, 0, 0, 0, accel_var};
    publisher_ = create_publisher<Imu>(output_topic, 50);
    subscription_ = create_subscription<Imu>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](Imu::ConstSharedPtr message) {relay(*message);});
    report_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(10.0), [this]() {report();});
  }

private:
  void relay(const Imu & message)
  {
    if (restamp_) {
      output_.header.stamp = get_clock()->now();
    } else {
      output_.header.stamp = message.header.stamp;
    }
    const auto & w = message.angular_velocity;
    const auto & a = message.linear_acceleration;
    const Axes gyro{w.x, w.y, w.z};
    const Axes accel{a.x * kGravity, a.y * kGravity, a.z * kGravity};
    const auto & bias = tracker_.bias();
    if (track_bias_ && tracker_.update(gyro, accel)) {
      const double change = std::max({
        std::abs(bias[0] - logged_bias_[0]),
        std::abs(bias[1] - logged_bias_[1]),
        std::abs(bias[2] - logged_bias_[2])});
      if (change > 2.0e-4) {
        logged_bias_ = bias;
        RCLCPP_INFO(
          get_logger(), "gyro bias now [%+.5f, %+.5f, %+.5f] rad/s (%zu still windows)",
          bias[0], bias[1], bias[2], tracker_.updates());
      }
    }
    output_.angular_velocity.x = gyro[0] - bias[0];
    output_.angular_velocity.y = gyro[1] - bias[1];
    output_.angular_velocity.z = gyro[2] - bias[2];
    output_.linear_acceleration.x = accel[0];
    output_.linear_acceleration.y = accel[1];
    output_.linear_acceleration.z = accel[2];
    publisher_->publish(output_);
    ++messages_;
  }

  void report()
  {
    const double rate = messages_ / 10.0;
    messages_ = 0;
    if (rate < 150.0) {
      RCLCPP_WARN(get_logger(), "livox imu relay: %.0f Hz (expected ~200)", rate);
    } else {
      RCLCPP_INFO(get_logger(), "livox imu relay: %.0f Hz", rate);
    }
  }

  Imu output_;
  bool restamp_;
  bool track_bias_;
  BiasTracker tracker_;
  Axes logged_bias_{};
  std::size_t messages_ = 0;
  rclcpp::Publisher<Imu>::SharedPtr publisher_;
  rclcpp::Subscription<Imu>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxImuRelay>());
  rclcpp::shutdown();
  return 0;
}
