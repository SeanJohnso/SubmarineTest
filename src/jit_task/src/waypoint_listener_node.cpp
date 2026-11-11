// waypoint_listener_node.cpp
//
// Receives a mission from another vessel and hands it to local_guided_node.
// Started only when the launch runs with waypoint_source:=topic.
//
//   in   jit/waypoints          nav_msgs/Path, in input_frame (the other
//                               vessel's "map")
//        /tf_static, /tf        input_frame -> local_frame, broadcast by the
//                               other vessel
//        /mavros/local_position/pose   for the geofence
//        jit/nav/local/status   to refuse a new list mid-mission
//   out  jit/nav/local/mission  jit_msgs/MissionXY, in local_frame, latched
//        jit/comms/waypoints/status    std_msgs/String, latched: "accepted N"
//                               or "rejected: <reason>", for the other vessel
//
// This node holds no MAVROS client, publishes no cmd/* topic and is not on the
// health bus. The worst it can do is fail to stage a mission, and then
// local_guided_node aborts on mode entry with "no mission received".
//
// ---------------------------------------------------------------------------
// NO DEPTH, BY CONSTRUCTION
// ---------------------------------------------------------------------------
// A topic-sourced mission must never command a dive. The output type,
// MissionXY, has no z field at all, so there is nothing here that could carry
// one: every z in the incoming Path is discarded, and local_guided_node holds
// the depth it captured at mode entry. A list from another vessel cannot command
// a depth under any waypoint_source.
//
// ---------------------------------------------------------------------------
// THE FINAL HEADING
// ---------------------------------------------------------------------------
// The orientation of the LAST pose is carried on MissionXY.final_yaw, rotated
// into local_frame by the same frozen transform as the positions. task_node
// uses it as the direction of the submerged traverse. Every other orientation
// in the list is ignored. A last pose with a zero-length or non-finite
// quaternion rejects the whole list, because it holds no direction at all.
//
// An orientation the sender never set is NOT caught. ROS message defaults make
// it the identity quaternion, which is a legal heading of 0 (East in input_frame)
// and is indistinguishable from a real one. The heading is logged on every
// accepted list so this can be seen.
//
// ---------------------------------------------------------------------------
// ALL OR NOTHING
// ---------------------------------------------------------------------------
// A list is accepted whole or rejected whole. Flying the valid half of a list
// that was partly wrong is a different mission from the one the sender meant.
// A rejected list leaves the previously accepted one in place.
//
// ---------------------------------------------------------------------------
// TRANSFORMED ONCE, THEN FROZEN
// ---------------------------------------------------------------------------
// The waypoints are transformed into local_frame on receipt, with the latest
// transform available, and the result is fixed. Nothing re-transforms them
// later. That matters twice over:
//
//   - vehicle_interface_node forwards a guided target only when it moves by
//     more than setpoint_epsilon_m, and every forward re-anchors ArduSub's leg
//     and restarts its s-curve. A target re-transformed through a jittering TF
//     would cross that threshold repeatedly and stall the vehicle - the exact
//     failure the setpoint dedupe was written to fix.
//   - the other vessel's TF stops arriving once the link is lost. A frozen
//     mission does not care.
//
// The lookup is at the latest time rather than at header.stamp, because the
// two vessels' clocks are not synchronised and a stamped lookup would fail on
// extrapolation for no real reason.
//
// ---------------------------------------------------------------------------
// NOT DURING A MISSION
// ---------------------------------------------------------------------------
// A new list is refused while local_guided_node reports RUNNING, and accepted
// in IDLE, COMPLETE or ABORTED. It is refused, not queued: the sender sees the
// rejection on jit/comms/waypoints/status and re-sends after completion.
//
// What an accepted list does next depends on waypoint_source. With "topic" it
// starts nothing: local_guided_node applies it at the next entry to
// LOCAL_GUIDED. With "task", task_node starts it as soon as it arrives, provided
// the switch is already in AUTO.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "jit_msgs/msg/mission_xy.hpp"
#include "jit_msgs/msg/waypoint_xy.hpp"

using MissionXY = jit_msgs::msg::MissionXY;
using WaypointXY = jit_msgs::msg::WaypointXY;

class WaypointListenerNode : public rclcpp::Node
{
public:
  WaypointListenerNode()
  : Node("waypoint_listener_node")
  {
    input_frame_ = this->declare_parameter<std::string>("input_frame", "map");
    local_frame_ = this->declare_parameter<std::string>("local_frame", "local_origin");
    // Geofence. Every waypoint must lie within max_waypoint_distance_m of the
    // sub's current position, horizontally, in local_frame.
    max_waypoints_ = this->declare_parameter<int>("max_waypoints", 50);
    max_waypoint_distance_m_ =
      this->declare_parameter<double>("max_waypoint_distance_m", 20.0);
    pose_timeout_s_ = this->declare_parameter<double>("pose_timeout_s", 1.0);
    // local_guided_node publishes its status every tick; silence past this
    // means its state is unknown, and an unknown state may be RUNNING.
    nav_status_timeout_s_ = this->declare_parameter<double>("nav_status_timeout_s", 1.0);

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    pose_stamp_ = nav_status_stamp_ = t0;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    mission_pub_ = this->create_publisher<MissionXY>("jit/nav/local/mission", latched);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "jit/comms/waypoints/status", latched);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "jit/waypoints", 10,
      std::bind(&WaypointListenerNode::path_cb, this, std::placeholders::_1));

    // SensorDataQoS: MAVROS publishes BEST_EFFORT, and a RELIABLE subscription
    // would silently receive nothing - every list would then be rejected for a
    // stale pose.
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        pose_ = *m;
        pose_stamp_ = this->now();
      });
    nav_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "jit/nav/local/status", 10,
      [this](const std_msgs::msg::String::SharedPtr m) {
        nav_status_ = m->data;
        nav_status_stamp_ = this->now();
      });

    RCLCPP_INFO(
      this->get_logger(),
      "waypoint_listener_node up. jit/waypoints in '%s' -> jit/nav/local/mission in '%s'. "
      "Geofence: %d waypoint(s), %.1f m from the current pose. z is discarded - topic "
      "missions are surface-only.",
      input_frame_.c_str(), local_frame_.c_str(), max_waypoints_, max_waypoint_distance_m_);
  }

private:
  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  void reject(const std::string & why)
  {
    RCLCPP_WARN(this->get_logger(), "Waypoint list REJECTED: %s", why.c_str());
    publish_status("rejected: " + why);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String s;
    s.data = text;
    status_pub_->publish(s);
  }

  void path_cb(const nav_msgs::msg::Path::SharedPtr path)
  {
    // ---- Is a new list allowed right now? ----
    if (!fresh(nav_status_stamp_, nav_status_timeout_s_)) {
      reject("local_guided_node status unknown (jit/nav/local/status stale)");
      return;
    }
    if (nav_status_ == "RUNNING") {
      reject("mission running - re-send after it completes");
      return;
    }

    // ---- Shape ----
    if (path->header.frame_id != input_frame_) {
      reject("frame_id '" + path->header.frame_id + "', expected '" + input_frame_ + "'");
      return;
    }
    if (path->poses.empty()) {
      reject("empty list");
      return;
    }
    if (static_cast<int>(path->poses.size()) > max_waypoints_) {
      reject(
        std::to_string(path->poses.size()) + " waypoints, limit " +
        std::to_string(max_waypoints_));
      return;
    }
    // A NaN would pass the geofence (NaN > limit is false), so check first.
    for (size_t i = 0; i < path->poses.size(); ++i) {
      const auto & p = path->poses[i].pose.position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        reject("waypoint " + std::to_string(i) + " has a non-finite x or y");
        return;
      }
    }

    // NaN or inf in any component carries through to the squared length.
    const auto & final_o = path->poses.back().pose.orientation;
    tf2::Quaternion final_q(final_o.x, final_o.y, final_o.z, final_o.w);
    const double final_q_len2 = final_q.length2();
    if (!std::isfinite(final_q_len2) || final_q_len2 < 1e-6) {
      reject("final waypoint has no valid heading");
      return;
    }
    final_q.normalize();

    // ---- Transform ----
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(local_frame_, input_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      reject("no transform " + input_frame_ + " -> " + local_frame_ + ": " + e.what());
      return;
    }
    const auto & r = tf_msg.transform.rotation;
    const auto & t = tf_msg.transform.translation;
    const tf2::Transform tf(
      tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(t.x, t.y, t.z));

    // ---- Geofence, against where the sub is now ----
    if (!fresh(pose_stamp_, pose_timeout_s_)) {
      reject("no fresh /mavros/local_position/pose for the geofence");
      return;
    }
    if (pose_.header.frame_id != local_frame_) {
      reject(
        "local pose is in '" + pose_.header.frame_id + "', expected '" + local_frame_ +
        "' - check the MAVROS frame_id parameter");
      return;
    }

    MissionXY mission;
    mission.header.stamp = this->now();
    mission.header.frame_id = local_frame_;
    for (size_t i = 0; i < path->poses.size(); ++i) {
      const auto & p = path->poses[i].pose.position;
      // z goes in as 0 and the transformed z is thrown away: depth is never
      // taken from this source.
      const tf2::Vector3 out = tf * tf2::Vector3(p.x, p.y, 0.0);
      const double d = std::hypot(
        out.x() - pose_.pose.position.x, out.y() - pose_.pose.position.y);
      if (d > max_waypoint_distance_m_) {
        char buf[160];
        std::snprintf(
          buf, sizeof(buf), "waypoint %zu is %.1f m from the sub, limit %.1f m",
          i, d, max_waypoint_distance_m_);
        reject(buf);
        return;
      }
      WaypointXY w;
      w.x = out.x();
      w.y = out.y();
      mission.waypoints.push_back(w);
    }

    double final_roll, final_pitch, final_yaw;
    tf2::Matrix3x3(tf.getRotation() * final_q).getRPY(final_roll, final_pitch, final_yaw);
    mission.final_yaw = final_yaw;

    mission_pub_->publish(mission);
    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint list ACCEPTED: %zu waypoint(s), %s -> %s, final heading %.1f deg ENU. "
      "Published on jit/nav/local/mission.",
      mission.waypoints.size(), input_frame_.c_str(), local_frame_.c_str(),
      final_yaw * 180.0 / M_PI);
    for (size_t i = 0; i < mission.waypoints.size(); ++i) {
      RCLCPP_INFO(
        this->get_logger(), "  wp[%zu]  x=%+.2f  y=%+.2f", i,
        mission.waypoints[i].x, mission.waypoints[i].y);
    }
    publish_status("accepted " + std::to_string(mission.waypoints.size()));
  }

  // --- Config ---
  std::string input_frame_;
  std::string local_frame_;
  int max_waypoints_ {50};
  double max_waypoint_distance_m_ {20.0};
  double pose_timeout_s_ {1.0};
  double nav_status_timeout_s_ {1.0};

  // --- Inputs ---
  geometry_msgs::msg::PoseStamped pose_;
  rclcpp::Time pose_stamp_;
  std::string nav_status_;
  rclcpp::Time nav_status_stamp_;

  // --- ROS interfaces ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<MissionXY>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointListenerNode>());
  rclcpp::shutdown();
  return 0;
}
