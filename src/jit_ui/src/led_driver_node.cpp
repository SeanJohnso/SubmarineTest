// led_driver_node.cpp
//
// Drives the indicator panel: a single-wire, WS281x-style addressable LED
// panel wired to one GPIO pin, driven with rpi_ws281x (PWM + DMA). Every
// pixel on the panel is set to the same solid colour - to the operator this
// is one three-colour light, not a display, matching LedCommand's three
// patterns.
//
// THIS NODE HOLDS NO SAFETY POLICY. That is unchanged from the stub this
// replaces. system_monitor_node is the single arbiter of what the panel
// shows - see its publish_led(). It checks the motor rail's actual relay
// feedback FIRST (Health::ESTOP_OPEN) and the granted mode second, which is
// exactly what makes the physical kill switch win: moving the 3-position RC
// switch to its ESTOP detent withdraws crsf/relay_permit, gpio_estop_node
// opens the relay, the feedback pin goes low, ESTOP_OPEN is raised, and the
// panel goes red - regardless of what mode was requested. This node must
// never re-derive that decision from health/mode/estop topics itself; it
// only translates the pattern string already decided upstream into a pixel
// colour.
//
//   ESTOP  -> RED     relay open (whatever the switch says) - see the header
//   MANUAL -> YELLOW  rail live, manual control (the RC "detent") - the
//                      operator-facing name for this is "RC"
//   AUTO   -> GREEN   rail live, a guided mode is active
//
// ---------------------------------------------------------------------------
// HARDWARE
// ---------------------------------------------------------------------------
// rpi_ws281x drives the PWM/PCM + DMA hardware directly: this process needs
// root (or the right capabilities), and only one process on the Pi may hold
// a given DMA channel at a time.
//
//   library: https://github.com/jgarff/rpi_ws281x - see README.MD for build
//   instructions. Not a ROS package / not colcon-built, so CMakeLists.txt
//   locates it manually, the same way jit_safety locates xcrsf and libgpiod.
//
// gpio_pin defaults to BCM13 (PWM channel 1, header pin 33) - deliberately
// NOT BCM18, which gpio_estop_node already owns as the relay feedback input.
// pwm_channel must match whichever hardware PWM channel gpio_pin belongs to
// (0: BCM12/BCM18, 1: BCM13/BCM19 - see the rpi_ws281x README). Both are
// parameters; the values here are reasonable, not measured, and must be
// checked against the real wiring before flying.
//
// The exact field names in ws2811.h vary a little between forks/versions of
// this library and could not be checked against an installed header in this
// environment (no Pi / no library available here) - verify field names
// compile clean on the Pi the first time this is built there, the same
// caveat crsf_channel_node.cpp carries for xcrsf.
//
// BOOT STATE. Until the latched led/command arrives, the panel is driven to
// RED. Same "I don't know -> assume unsafe" convention used everywhere else
// in this stack (gpio_estop_node's relay starts commanded open;
// system_monitor_node's relay permit starts denied). The latched QoS below
// means that in practice this is overwritten within one subscription
// handshake of system_monitor_node already running.

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "jit_msgs/msg/led_command.hpp"

extern "C" {
#include "ws2811.h"
}

namespace
{
int parse_strip_type(const std::string & s)
{
  if (s == "RGB") {return WS2811_STRIP_RGB;}
  if (s == "RBG") {return WS2811_STRIP_RBG;}
  if (s == "GRB") {return WS2811_STRIP_GRB;}
  if (s == "GBR") {return WS2811_STRIP_GBR;}
  if (s == "BRG") {return WS2811_STRIP_BRG;}
  if (s == "BGR") {return WS2811_STRIP_BGR;}
  return WS2811_STRIP_GRB;  // WS2812B's usual wire order
}

constexpr uint32_t COLOUR_RED = 0xFF0000;
constexpr uint32_t COLOUR_YELLOW = 0xFFFF00;
constexpr uint32_t COLOUR_GREEN = 0x00FF00;
constexpr uint32_t COLOUR_OFF = 0x000000;
}  // namespace

class LedDriverNode : public rclcpp::Node
{
public:
  LedDriverNode()
  : Node("led_driver_node")
  {
    const int gpio_pin = this->declare_parameter<int>("gpio_pin", 13);
    const int pwm_channel = this->declare_parameter<int>("pwm_channel", 1);
    const int dma_channel = this->declare_parameter<int>("dma_channel", 10);
    const int led_count = this->declare_parameter<int>("led_count", 256);
    const int freq_hz = this->declare_parameter<int>("freq_hz", 800000);
    const int brightness = this->declare_parameter<int>("brightness", 64);
    const bool invert = this->declare_parameter<bool>("invert", false);
    const std::string strip_type_str =
      this->declare_parameter<std::string>("strip_type", "GRB");

    if (pwm_channel != 0 && pwm_channel != 1) {
      throw std::runtime_error("pwm_channel must be 0 or 1");
    }
    if (led_count <= 0) {
      throw std::runtime_error("led_count must be positive");
    }
    if (brightness < 0 || brightness > 255) {
      throw std::runtime_error("brightness must be in [0, 255]");
    }

    strip_ = ws2811_t{};
    strip_.freq = static_cast<uint32_t>(freq_hz);
    strip_.dmanum = dma_channel;
    strip_.channel[pwm_channel].gpionum = gpio_pin;
    strip_.channel[pwm_channel].invert = invert ? 1 : 0;
    strip_.channel[pwm_channel].count = led_count;
    strip_.channel[pwm_channel].brightness = static_cast<uint8_t>(brightness);
    strip_.channel[pwm_channel].strip_type = parse_strip_type(strip_type_str);
    active_channel_ = pwm_channel;
    led_count_ = led_count;

    const ws2811_return_t init_ret = ws2811_init(&strip_);
    if (init_ret != WS2811_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ws2811_init failed on GPIO%d (DMA %d, PWM ch %d): %s. Panel will not light - "
        "check that nothing else holds this DMA channel and that this process has the "
        "permissions rpi_ws281x needs (root, or the right capabilities).",
        gpio_pin, dma_channel, pwm_channel, ws2811_get_return_t_str(init_ret));
      throw std::runtime_error("ws2811_init failed");
    }
    initialized_ = true;

    set_all(COLOUR_RED);
    render("boot (assumed ESTOP until led/command arrives)");

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    sub_ = this->create_subscription<jit_msgs::msg::LedCommand>(
      "led/command", latched,
      std::bind(&LedDriverNode::led_cb, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "led_driver_node up: %d-pixel panel on GPIO%d (PWM ch %d, DMA %d, %s order).",
      led_count, gpio_pin, pwm_channel, dma_channel, strip_type_str.c_str());
  }

  ~LedDriverNode() override
  {
    if (initialized_) {
      set_all(COLOUR_OFF);
      render("shutdown");
      ws2811_fini(&strip_);
    }
  }

private:
  void led_cb(const jit_msgs::msg::LedCommand::SharedPtr msg)
  {
    if (msg->pattern == pattern_) {
      return;
    }
    pattern_ = msg->pattern;

    uint32_t colour = COLOUR_RED;
    const char * shown = "ESTOP (red)";
    if (pattern_ == jit_msgs::msg::LedCommand::MANUAL) {
      colour = COLOUR_YELLOW;
      shown = "MANUAL/RC (yellow)";
    } else if (pattern_ == jit_msgs::msg::LedCommand::AUTO) {
      colour = COLOUR_GREEN;
      shown = "AUTO (green)";
    } else if (pattern_ != jit_msgs::msg::LedCommand::ESTOP) {
      RCLCPP_WARN(
        this->get_logger(), "Unknown led/command pattern '%s' - showing RED/ESTOP.",
        pattern_.c_str());
    }

    set_all(colour);
    render(shown);
  }

  void set_all(uint32_t colour)
  {
    ws2811_led_t * const leds = strip_.channel[active_channel_].leds;
    for (int i = 0; i < led_count_; ++i) {
      leds[i] = colour;
    }
  }

  void render(const char * why)
  {
    const ws2811_return_t ret = ws2811_render(&strip_);
    if (ret != WS2811_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "ws2811_render failed (%s): %s", why, ws2811_get_return_t_str(ret));
      return;
    }
    RCLCPP_INFO(this->get_logger(), "LED panel -> %s.", why);
  }

  ws2811_t strip_{};
  int active_channel_ {0};
  int led_count_ {0};
  bool initialized_ {false};
  std::string pattern_;
  rclcpp::Subscription<jit_msgs::msg::LedCommand>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LedDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("led_driver_node"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}