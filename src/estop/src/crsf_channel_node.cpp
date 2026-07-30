// crsf_channel_node.cpp
//
// Reads CRSF/ELRS channel data from a T8L-paired receiver via the xcrsf
// library and publishes:
//   - crsf/channels            (std_msgs/UInt16MultiArray)  all 16 raw channels
//   - crsf/channel_threshold   (std_msgs/Bool)               threshold_check_callback() result
//
// The threshold_check_callback() function is the hook you asked for: it's
// where a single channel gets compared against a threshold and turned into
// a bool. A separate node (not this one) should subscribe to
// crsf/channel_threshold and actually drive the GPIO pin — keeping "decode
// CRSF" and "touch hardware GPIO" as two independent, individually
// testable nodes.
//
// Verified against the real installed header (/usr/local/include/xcrsf/crossfire.h):
// open_port(), is_paired(), get_channel_state() -> std::array<uint16_t, 16>, get_link_state().

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "xcrsf/crossfire.h"

using namespace std::chrono_literals;

class CrsfChannelNode : public rclcpp::Node
{
public:
  CrsfChannelNode()
  : Node("crsf_channel_node"), port_open_(false)
  {
    // --- Parameters (set these via a launch file or `ros2 run ... --ros-args -p`) ---
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    poll_rate_hz_ = this->declare_parameter<double>("poll_rate_hz", 50.0);

    // Threshold-check config. threshold_channel_index is 0-based and
    // defaults to -1 (disabled) until you know which channel is your
    // toggle — find that with crsf_test.py first, then set it here.
    threshold_channel_index_ = this->declare_parameter<int>("threshold_channel_index", -1);
    threshold_value_ = this->declare_parameter<int>("threshold_value", 1500);
    threshold_above_triggers_ = this->declare_parameter<bool>("threshold_above_triggers", true);

    // --- Publishers ---
    channels_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
      "crsf/channels", 10);
    threshold_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "crsf/channel_threshold", 10);

    // --- Open the CRSF serial link ---
    // ADJUST HERE if the real constructor doesn't take a plain C string.
    crossfire_ = std::make_unique<crossfire::XCrossfire>(serial_port_.c_str());
    port_open_ = crossfire_->open_port();
    if (port_open_) {
      RCLCPP_INFO(this->get_logger(), "Opened CRSF port '%s'", serial_port_.c_str());
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open CRSF port '%s' — will keep retrying every poll cycle.",
        serial_port_.c_str());
    }

    // --- Poll timer ---
    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CrsfChannelNode::poll_callback, this));
  }

private:
  void poll_callback()
  {
    if (!port_open_) {
      port_open_ = crossfire_->open_port();
      if (!port_open_) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "CRSF port recovered.");
    }

    if (!crossfire_->is_paired()) {
      // No valid link this cycle — publish nothing rather than stale/zero data.
      return;
    }

    // Confirmed against the real header: returns std::array<uint16_t, 16>.
    const std::array<uint16_t, 16> channels = crossfire_->get_channel_state();

    std_msgs::msg::UInt16MultiArray channels_msg;
    channels_msg.data.assign(channels.begin(), channels.end());
    channels_pub_->publish(channels_msg);

    threshold_check_callback(channels);
  }

  // --- Threshold check / boolean publish hook ---
  // This is the callback you can tie into any channel with a threshold.
  // Right now it does a generic above/below comparison against whichever
  // channel index and value you set via ROS parameters, and publishes the
  // result as std_msgs/Bool on crsf/channel_threshold. Replace the body
  // with whatever your toggle actually needs (hysteresis, debounce,
  // multiple channels, etc) — the publisher and topic stay the same so
  // nothing downstream has to change.
  template <typename ChannelContainer>
  void threshold_check_callback(const ChannelContainer & channels)
  {
    if (threshold_channel_index_ < 0 ||
      static_cast<size_t>(threshold_channel_index_) >= channels.size())
    {
      // Not configured yet — set the threshold_channel_index parameter
      // once you've identified your toggle channel with crsf_test.py.
      return;
    }

    const uint16_t value = channels[threshold_channel_index_];
    const bool triggered = threshold_above_triggers_
      ? (value > threshold_value_)
      : (value < threshold_value_);

    std_msgs::msg::Bool bool_msg;
    bool_msg.data = triggered;
    threshold_pub_->publish(bool_msg);
  }

  // --- CRSF state ---
  std::unique_ptr<crossfire::XCrossfire> crossfire_;
  bool port_open_;
  std::string serial_port_;
  double poll_rate_hz_;

  // --- Threshold config ---
  int threshold_channel_index_;
  int threshold_value_;
  bool threshold_above_triggers_;

  // --- ROS interfaces ---
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr channels_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr threshold_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrsfChannelNode>());
  rclcpp::shutdown();
  return 0;
}