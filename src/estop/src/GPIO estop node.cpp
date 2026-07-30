// gpio_estop_node.cpp
//
// Subscribes to crsf/channel_threshold (published by crsf_channel_node)
// and drives a single GPIO pin as a software E-stop:
//
//   - On startup: pin is set HIGH (motors enabled / relay not tripped).
//   - When crsf/channel_threshold is true (channel 5 above 1500):
//       pin goes LOW -> trips the NO relay -> motors safed.
//   - When it goes back to false: pin returns HIGH.
//   - If this node dies or the topic stops publishing, the pin simply
//     stays at whatever it was last set to. If you want a true "no
//     heartbeat = tripped" fail-safe, that needs a watchdog timer added
//     here later (not implemented yet — flag this if you want it).
//
// Current status is NOT kept as a private member you'd need to query
// internally — it's published on estop/status (std_msgs/Bool) with a
// TRANSIENT_LOCAL ("latched") QoS. That means any node that subscribes
// later — like a future LED panel node — gets the current state
// immediately on subscribing, without needing to wait for the next
// state change or reach into this node's internals.
//
// GPIO is done via libgpiod (the modern kernel gpio-cdev interface),
// not the deprecated sysfs /sys/class/gpio approach.
//
// IMPORTANT: gpio_line_offset defaults to 17 (BCM GPIO17 / physical pin
// 11) as a placeholder. Set it to whatever pin your relay is actually
// wired to via the ROS parameter. Do NOT use offset 0 or 1 - those are
// reserved (ID_SD/ID_SC EEPROM pins), not general-purpose.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

extern "C" {
#include <gpiod.h>
}

class GpioEstopNode : public rclcpp::Node
{
public:
  GpioEstopNode()
  : Node("gpio_estop_node"), chip_(nullptr), line_(nullptr), estop_active_(false)
  {
    chip_name_ = this->declare_parameter<std::string>("gpio_chip", "gpiochip0");
    line_offset_ = this->declare_parameter<int>("gpio_line_offset", 17);

    // --- Open the GPIO chip and request the line as an output ---
    chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
    if (!chip_) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to open GPIO chip '%s'. Is libgpiod set up correctly?",
        chip_name_.c_str());
      throw std::runtime_error("gpiod_chip_open_by_name failed");
    }

    line_ = gpiod_chip_get_line(chip_, line_offset_);
    if (!line_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get GPIO line offset %d", line_offset_);
      throw std::runtime_error("gpiod_chip_get_line failed");
    }

    // Request as output, default value 1 (HIGH) -> motors enabled at startup.
    const int request_result = gpiod_line_request_output(line_, "gpio_estop_node", 1);
    if (request_result < 0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Failed to request GPIO line %d as output. Is another process using it, "
        "or do you need to be in the 'gpio' group / run with sudo?",
        line_offset_);
      throw std::runtime_error("gpiod_line_request_output failed");
    }

    RCLCPP_INFO(
      this->get_logger(), "GPIO %s line %d requested, set HIGH at startup (motors enabled).",
      chip_name_.c_str(), line_offset_);

    // --- Status publisher: latched so late subscribers (e.g. an LED
    // panel node) immediately get the current state on connect. ---
    rclcpp::QoS status_qos(1);
    status_qos.reliable();
    status_qos.transient_local();
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("estop/status", status_qos);

    // Publish the initial state right away (estop_active_ = false at startup).
    publish_status();

    // --- Subscribe to the threshold bool from crsf_channel_node ---
    threshold_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "crsf/channel_threshold", 10,
      std::bind(&GpioEstopNode::threshold_callback, this, std::placeholders::_1));
  }

  ~GpioEstopNode() override
  {
    if (line_) {
      // Leave the relay in a safe (tripped) state on shutdown rather than
      // whatever it last happened to be — comment this out if you'd
      // rather it hold its last state instead.
      gpiod_line_set_value(line_, 0);
      gpiod_line_release(line_);
    }
    if (chip_) {
      gpiod_chip_close(chip_);
    }
  }

private:
  void threshold_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool new_estop_active = msg->data;

    if (new_estop_active == estop_active_) {
      return;  // no change, nothing to do
    }

    estop_active_ = new_estop_active;

    // estop_active_ true -> trip relay -> pin LOW.
    // estop_active_ false -> motors enabled -> pin HIGH.
    const int gpio_value = estop_active_ ? 0 : 1;
    const int set_result = gpiod_line_set_value(line_, gpio_value);
    if (set_result < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO line %d", line_offset_);
    }

    RCLCPP_WARN_EXPRESSION(
      this->get_logger(), estop_active_,
      "E-STOP TRIPPED: channel threshold exceeded, GPIO %d set LOW.", line_offset_);
    RCLCPP_INFO_EXPRESSION(
      this->get_logger(), !estop_active_,
      "E-stop cleared: channel back to nominal, GPIO %d set HIGH.", line_offset_);

    publish_status();
  }

  void publish_status()
  {
    std_msgs::msg::Bool status_msg;
    status_msg.data = estop_active_;
    status_pub_->publish(status_msg);
  }

  // --- GPIO ---
  std::string chip_name_;
  int line_offset_;
  gpiod_chip * chip_;
  gpiod_line * line_;

  // --- State ---
  bool estop_active_;

  // --- ROS interfaces ---
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr threshold_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GpioEstopNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gpio_estop_node"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}