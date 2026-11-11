// heartbeat_node.cpp
//
// Publishes the vehicle's status for other machines on the network, at 2 Hz.
//
//   in   /mavros/global_position/global      position (lat/lon)
//        /mavros/local_position/pose         heading, roll, pitch
//        /mavros/local_position/velocity_local   speed
//        jit/mode, jit/health                state
//        jit/task/active                     current task
//   out  /robotx_comm/heartbeat   robotx_comm/msg/Heartbeat
//
// Read-only: this node holds no MAVROS client, publishes no cmd/* topic and is
// not on the health bus, so it cannot move the vehicle.
//
// ---------------------------------------------------------------------------
// THE PACKAGE NAME IS PART OF THE INTERFACE
// ---------------------------------------------------------------------------
// ROS 2 matches a publisher to a subscriber on the full type name as well as
// the topic. Downstream subscribers expect robotx_comm/msg/Heartbeat, so the
// package name, and the field names, types and order in Heartbeat.msg, must not
// change. The topic is outside jit/ for the same reason.
//
// ---------------------------------------------------------------------------
// GLOBAL POSITION UNDERWATER
// ---------------------------------------------------------------------------
// The lat/lon on /mavros/global_position/global are the EKF's estimate, while
// its status field is copied from the raw GPS. Underwater it can read NO_FIX
// while the DVL is still carrying a good position, so status is ignored here.
// ArduPilot also sends this position when the EKF has marked it invalid, so a
// frozen position looks normal on this topic. Before the EKF has an origin the
// lat/lon are 0, 0.
//
// ---------------------------------------------------------------------------
// UNVERIFIED ON THE VEHICLE
// ---------------------------------------------------------------------------
// velocity_local is assumed to be published. This node waits for it, so if it
// is missing no heartbeat goes out and the throttled log names it.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "jit_msgs/msg/health.hpp"
#include "jit_msgs/msg/mode.hpp"
#include "robotx_comm/msg/heartbeat.hpp"

using Health = jit_msgs::msg::Health;
using Mode = jit_msgs::msg::Mode;
using Heartbeat = robotx_comm::msg::Heartbeat;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TwistStamped = geometry_msgs::msg::TwistStamped;

namespace
{
// Values shared with downstream subscribers. Kept here so Heartbeat.msg stays
// identical to the original definition.
constexpr uint8_t STATE_UNKNOWN = 0;
constexpr uint8_t STATE_KILLED = 1;
constexpr uint8_t STATE_MANUAL = 2;
constexpr uint8_t STATE_AUTO = 3;

constexpr uint8_t TASK_NONE = 1;
constexpr uint8_t TASK_INFRA_SURVEY_REPAIR = 3;

constexpr uint8_t TYPE_UUV = 2;

constexpr double RAD2DEG = 180.0 / M_PI;
constexpr auto PUBLISH_PERIOD = std::chrono::milliseconds(500);
}  // namespace

class HeartbeatNode : public rclcpp::Node
{
public:
  HeartbeatNode()
  : Node("heartbeat_node")
  {
    vehicle_id_ = this->declare_parameter<std::string>("vehicle_id", "4");
    frame_ = this->declare_parameter<std::string>("frame", "ENU");
    // jit/mode is published at 10 Hz; silence past this means state is unknown.
    mode_timeout_s_ = this->declare_parameter<double>("mode_timeout_s", 1.0);

    if (frame_ != "ENU" && frame_ != "NED") {
      RCLCPP_WARN(
        this->get_logger(), "Invalid frame '%s'. Valid choices: ENU, NED. Using ENU.",
        frame_.c_str());
      frame_ = "ENU";
    }

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    heartbeat_pub_ = this->create_publisher<Heartbeat>(
      "/robotx_comm/heartbeat", rclcpp::SensorDataQoS());

    // SensorDataQoS: MAVROS publishes BEST_EFFORT, and a RELIABLE subscription
    // would silently receive nothing.
    global_sub_ = this->create_subscription<NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](const NavSatFix::SharedPtr m) {global_ = m;});
    pose_sub_ = this->create_subscription<PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const PoseStamped::SharedPtr m) {pose_ = m;});
    velocity_sub_ = this->create_subscription<TwistStamped>(
      "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
      [this](const TwistStamped::SharedPtr m) {velocity_ = m;});

    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {
        mode_ = m;
        mode_stamp_ = this->now();
      });
    health_sub_ = this->create_subscription<Health>(
      "jit/health", latched,
      [this](const Health::SharedPtr m) {faults_ = m->faults;});

    // The only input to current_task. task_node sets it the moment it takes a
    // list and clears it once the vehicle has stabilized on the surface. Only
    // waypoint_source:=task has a publisher, so other sources report TASK_NONE.
    task_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "jit/task/active", latched,
      [this](const std_msgs::msg::Bool::SharedPtr m) {
        current_task_ = m->data ? TASK_INFRA_SURVEY_REPAIR : TASK_NONE;
      });

    timer_ = this->create_wall_timer(
      PUBLISH_PERIOD, std::bind(&HeartbeatNode::publish_heartbeat, this));

    RCLCPP_INFO(
      this->get_logger(),
      "heartbeat_node up. Publishing /robotx_comm/heartbeat at 2 Hz as id '%s', frame %s, "
      "once position, attitude, velocity and mode have arrived.",
      vehicle_id_.c_str(), frame_.c_str());
  }

private:
  uint8_t vehicle_state() const
  {
    if ((this->now() - mode_stamp_).seconds() > mode_timeout_s_) {
      return STATE_UNKNOWN;
    }
    // ESTOP_OPEN first, as in system_monitor_node::publish_led(): the relay's
    // real feedback wins over the granted mode.
    if ((faults_ & Health::ESTOP_OPEN) || mode_->mode == Mode::SAFE) {
      return STATE_KILLED;
    }
    if (mode_->mode == Mode::MANUAL) {
      return STATE_MANUAL;
    }
    if (mode_->mode == Mode::LOCAL_GUIDED) {
      return STATE_AUTO;
    }
    return STATE_UNKNOWN;
  }

  void publish_heartbeat()
  {
    if (!global_ || !pose_ || !velocity_ || !mode_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Heartbeat waiting for:%s%s%s%s",
        global_ ? "" : " global_position/global",
        pose_ ? "" : " local_position/pose",
        velocity_ ? "" : " local_position/velocity_local",
        mode_ ? "" : " jit/mode");
      return;
    }

    Heartbeat msg;
    msg.time = this->now();
    msg.id = vehicle_id_;
    msg.seq = seq_++;
    msg.state = vehicle_state();
    msg.position.latitude = global_->latitude;
    msg.position.longitude = global_->longitude;

    const auto & v = velocity_->twist.linear;
    msg.spd_mps = static_cast<float>(std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));

    const auto & q = pose_->pose.orientation;
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    double roll_deg = roll * RAD2DEG;
    double pitch_deg = pitch * RAD2DEG;
    double yaw_deg = yaw * RAD2DEG;
    if (frame_ == "NED") {
      yaw_deg = std::fmod(90.0 - yaw_deg + 360.0, 360.0);
      pitch_deg = -pitch_deg;
    }
    msg.heading_deg = static_cast<float>(yaw_deg);
    msg.roll_deg = static_cast<float>(roll_deg);
    msg.pitch_deg = static_cast<float>(pitch_deg);

    // altitude_hae_m is intentionally not filled in.
    msg.depth_m = 0.0F;
    msg.current_task = current_task_;
    msg.vehicle_type = TYPE_UUV;
    msg.flight_phase = 0;
    msg.uav_registration = "";

    heartbeat_pub_->publish(msg);
  }

  // --- Config ---
  std::string vehicle_id_;
  std::string frame_;
  double mode_timeout_s_ {1.0};

  // --- Inputs ---
  NavSatFix::SharedPtr global_;
  PoseStamped::SharedPtr pose_;
  TwistStamped::SharedPtr velocity_;
  Mode::SharedPtr mode_;
  rclcpp::Time mode_stamp_;
  uint32_t faults_ {0};
  uint8_t current_task_ {TASK_NONE};
  uint32_t seq_ {0};

  // --- ROS interfaces ---
  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<NavSatFix>::SharedPtr global_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_active_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatNode>());
  rclcpp::shutdown();
  return 0;
}
