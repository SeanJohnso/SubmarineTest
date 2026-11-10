// local_guided_node.cpp
//
// Waypoint tracking against the DVL-backed local pose, for ArduSub GUIDED mode.
// Publishes jit/cmd/local_guided/setpoint (geometry_msgs/PoseStamped), which
// vehicle_interface_node forwards to /mavros/setpoint_position/local. This node
// holds no MAVROS client and cannot arm anything.
//
// ---------------------------------------------------------------------------
// WHY ACCEPTANCE LOGIC LIVES HERE
// ---------------------------------------------------------------------------
// ArduSub's GUIDED mode never declares a waypoint reached. Verified in the
// firmware source: guided_pos_control_run() calls wp_nav.update_wpnav() and
// never checks reached_wp_destination(). A position target in GUIDED is a
// station to hold, not a task to complete - the vehicle drives to it and holds
// there indefinitely. Only AUTO mode sequences waypoints, and AUTO needs
// lat/lon mission items and an EKF origin that can be set only once per power
// cycle, which does not fit the capture-a-datum-at-mode-entry approach below.
//
// So we do the acceptance test ourselves, deliberately mirroring ArduPilot's
// verify_nav_wp():
//
//   1. once the horizontal distance to the target is <= radius_m, latch
//      "reached" and stamp the time;
//   2. wait dwell_s from that stamp, then advance.
//
// The distance is NOT re-checked during the dwell - that matches the firmware,
// and it is the reason a tight radius is workable at all: the vehicle only has
// to touch the ball once, not stay inside it.
//
// A second consequence of the missing timeout matters for safety: stopping the
// setpoint stream does NOT stop the vehicle. Only leaving GUIDED does. That is
// vehicle_interface_node's job during safing.
//
// ---------------------------------------------------------------------------
// DATUM, DEPTH AND HEADING
// ---------------------------------------------------------------------------
// On entry to LOCAL_GUIDED the node captures the current pose as the datum and
// the current depth as entry_z. Every waypoint is an OFFSET from that datum, so
// estimator drift accumulated since boot is absorbed at the moment the mission
// starts rather than being baked into absolute coordinates.
//
// Offsets are along the local ENU frame's axes (x = East, y = North), NOT
// rotated into the vehicle's heading at entry. A waypoint of {x: 2, y: 0} is
// two metres East of wherever the sub was, whichever way it was pointing.
//
// z defaults to entry_z - the vehicle holds the depth it started at, so it does
// not fight the trim changes seen during testing. A waypoint may override it
// with its own `z`, which is also an offset (negative = deeper, ENU).
//
// THE DEPTH OVERRIDE IS STICKY. Once a waypoint sets z, every later waypoint
// that omits z holds THAT depth, not entry_z. Without this, the intended
// mission structure - one dive waypoint followed by level translation
// waypoints - would command the sub back to the surface on the first
// translation leg, because those legs carry no z of their own. z_offset_for()
// is the single definition: walk the list to the index and take the last
// override seen. It is a pure function of the index, so there is no sticky
// state to get out of sync with a waypoint advance.
//
// Heading points along track: the bearing from the current position to the
// target. It is not an input - no waypoint carries an orientation - and it is
// never used to decide whether a target is new. vehicle_interface_node forwards
// on a position change alone, so the bearing that actually reaches the vehicle
// is always the one sampled at a waypoint advance, with the new target a full
// leg away. A value that is never sent from close in needs no close-in guard.
//
// ---------------------------------------------------------------------------
// ACCEPTANCE AND THE VERTICAL AXIS
// ---------------------------------------------------------------------------
// The horizontal test above is not sufficient on its own for a leg that
// changes depth. A waypoint directly below the datum has a horizontal distance
// of ~0 from the instant the mission starts, so a horizontal-only test latches
// "reached" immediately, dwells, and completes the mission without the vehicle
// ever descending.
//
// So the vertical axis is checked too, but ONLY on a leg that actually commands
// a depth change (see leg_changes_depth). On a level leg the depth error is
// ignored exactly as before, which matters because holding entry_z on the
// surface carries a persistent trim error that would otherwise block every
// acceptance forever.
//
// ---------------------------------------------------------------------------
// nav/local/submerged - RELAXING THE RC LINK-LOSS FAILSAFE
// ---------------------------------------------------------------------------
// Submerging the vehicle submerges the ELRS antenna, which drops the RC link.
// Normally that is a fault: crsf_channel_node withdraws the relay permit and
// system_monitor_node raises RC_LINK_LOST, which is inside FAULT_SAFE_MASK.
// Both open the motor relay, so the vehicle cannot complete a submerged
// mission under power.
//
// This node therefore publishes nav/local/submerged: a statement of fact -
// "the mission is currently below the surface" - not a command. Two consumers
// decide policy from it, and each applies its own freshness check:
//
//   crsf_channel_node    holds crsf/relay_permit true and holds mode/request
//                        at the last live detent, but ONLY while the link is
//                        actually down. A live link means the operator's ESTOP
//                        detent works normally.
//   system_monitor_node  does not raise RC_LINK_LOST. CRSF_NODE_DEAD,
//                        MAVROS_LOST and ESTOP_NODE_DEAD are untouched.
//
// PUBLISH-ALWAYS, like crsf/link_ok. The flag goes out on every tick whatever
// the state, so silence on this topic means this process died - at which point
// both consumers time out within submerged_timeout_s and the failsafe re-arms.
//
// WHILE SUBMERGED THERE IS NO OPERATOR STOP. The switch is unreachable. What
// still stops the vehicle is: this node's max_submerged_s deadman, the
// independent backstop timer inside crsf_channel_node, MAVROS_LOST, the death
// of any of the three safety processes, and the mission completing. That is
// the whole list, and it is why the deadman is not optional.
//
// The flag is derived from the COMMANDED depth or the MEASURED depth, not from
// "the current waypoint has a z". Commanded, because it must be asserted before
// the descent begins rather than after the antenna is already under. Measured,
// because an ascent leg commands a shallow depth while the vehicle is still
// deep, and dropping the flag there would cut the motor rail halfway up.
//
// ---------------------------------------------------------------------------
// RE-ENTRY
// ---------------------------------------------------------------------------
// Leaving and re-entering LOCAL_GUIDED always recaptures the datum and always
// restarts at index 0. The operator's re-run gesture is therefore a single
// switch cycle. This is intentional for single-waypoint testing; it does mean
// that intervening mid-mission and returning walks the whole pattern.
//
// ---------------------------------------------------------------------------
// WAYPOINT SOURCE: file OR topic
// ---------------------------------------------------------------------------
// waypoint_source picks where the mission comes from, once, at launch:
//
//   file   waypoint_file, as described above. Offsets from the datum, and the
//          only source that may command a depth change.
//   topic  jit/nav/local/mission (jit_msgs/MissionXY) from
//          waypoint_listener_node, which received it from another vessel and
//          transformed it into local_frame. These waypoints are ABSOLUTE x/y in
//          local_frame, so the datum is not added to them.
//
// A topic mission CANNOT DIVE, by construction: MissionXY has no z field, so
// every waypoint enters this node with has_z = false and z_offset_for() holds
// entry_z for the whole run. A topic mission is surface-only, and therefore
// never asserts nav/local/submerged by command.
//
// The latest mission is cached as it arrives but only takes effect at mode
// entry, when it is copied into waypoints_. A mission arriving mid-run cannot
// change the running one (the listener refuses to send one while this node
// reports RUNNING anyway). A new list therefore always needs the operator's
// switch cycle to start - another vessel can stage a mission but cannot start
// one. Re-entry with no new list re-flies the last one, as the file does.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "jit_msgs/msg/mission_xy.hpp"
#include "jit_msgs/msg/mode.hpp"

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
using Mode = jit_msgs::msg::Mode;
using MissionXY = jit_msgs::msg::MissionXY;

namespace
{
struct Waypoint
{
  double x {0.0};        // offset East from the datum, metres (topic: absolute East)
  double y {0.0};        // offset North from the datum, metres (topic: absolute North)
  double z {0.0};        // offset Up from entry_z, metres (negative = deeper)
  bool has_z {false};    // false -> hold the depth of the previous leg
};

// Two commanded depths closer than this are the same depth, so the leg between
// them is a level leg and its acceptance test stays horizontal-only. Not a
// parameter: it distinguishes "this leg changes depth" from floating-point
// noise, and has nothing to do with how accurately a depth must be held.
constexpr double kDepthLegEpsilon = 1e-3;
}  // namespace

class LocalGuidedNode : public rclcpp::Node
{
public:
  LocalGuidedNode()
  : Node("local_guided_node")
  {
    waypoint_source_ = this->declare_parameter<std::string>("waypoint_source", "file");
    waypoint_file_ = this->declare_parameter<std::string>("waypoint_file", "");
    // The sub's local frame: MAVROS stamps local_position/pose with it, every
    // setpoint is stamped with it, and a topic mission must arrive in it.
    local_frame_ = this->declare_parameter<std::string>("local_frame", "local_origin");
    radius_m_ = this->declare_parameter<double>("radius_m", 0.3);
    dwell_s_ = this->declare_parameter<double>("dwell_s", 1.0);
    setpoint_rate_hz_ = this->declare_parameter<double>("setpoint_rate_hz", 10.0);
    pose_timeout_s_ = this->declare_parameter<double>("pose_timeout_s", 1.0);
    bus_timeout_s_ = this->declare_parameter<double>("bus_timeout_s", 0.5);

    // Vertical acceptance, applied only on a leg that commands a depth change.
    depth_radius_m_ = this->declare_parameter<double>("depth_radius_m", 0.3);

    // How far below the entry depth counts as submerged, for nav/local/submerged.
    // Asserting it early is free - the flag does nothing unless the RC link is
    // actually down - so this wants to be shallower than the depth at which the
    // antenna goes under, not deeper.
    submerge_threshold_m_ = this->declare_parameter<double>("submerge_threshold_m", 0.2);

    // DEADMAN. The longest this node will keep asserting nav/local/submerged in
    // one continuous stretch. While that flag is asserted and the link is down
    // the operator has no stop, so this timer is the primary bound on the whole
    // behaviour. On expiry the flag drops and the mission ABORTs, which cuts the
    // setpoints, re-arms the RC failsafe and lets the vehicle float up.
    // 0 disables the deadman - do not do that on a real dive.
    max_submerged_s_ = this->declare_parameter<double>("max_submerged_s", 120.0);

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    pose_stamp_ = mode_stamp_ = t0;

    topic_source_ = (waypoint_source_ == "topic");
    if (topic_source_) {
      RCLCPP_INFO(
        this->get_logger(),
        "waypoint_source=topic: missions come from jit/nav/local/mission, surface-only. "
        "waypoint_file is ignored.");
    } else {
      if (waypoint_source_ != "file") {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unknown waypoint_source '%s' (expected 'file' or 'topic'). Falling back to file.",
          waypoint_source_.c_str());
      }
      load_waypoints();
    }

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "jit/cmd/local_guided/setpoint", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("jit/nav/local/status", 10);

    // Publish-always: goes out every tick whatever the state, so that silence
    // on this topic means this process died rather than "not submerged". Both
    // consumers re-arm the RC failsafe on staleness. See the header.
    submerged_pub_ = this->create_publisher<std_msgs::msg::Bool>("jit/nav/local/submerged", 10);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    // SensorDataQoS (best-effort): MAVROS publishes its sensor topics
    // BEST_EFFORT, and a default RELIABLE subscription silently receives
    // NOTHING from it - rmw reports "incompatible QoS" and drops every sample.
    // The mission would then always abort with "no fresh pose", which is the
    // right safety behaviour for entirely the wrong reason.
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        pose_ = *m;
        have_pose_ = true;
        pose_stamp_ = this->now();
      });
    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {granted_mode_ = m->mode; mode_stamp_ = this->now();});

    // Latched on both ends, so a mission published before this node (re)started
    // still arrives.
    if (topic_source_) {
      mission_sub_ = this->create_subscription<MissionXY>(
        "jit/nav/local/mission", latched,
        std::bind(&LocalGuidedNode::mission_cb, this, std::placeholders::_1));
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / setpoint_rate_hz_)),
      std::bind(&LocalGuidedNode::tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "local_guided_node up. %zu waypoint(s) from '%s'. Acceptance: %.2f m horizontal "
      "(plus %.2f m vertical on a depth-changing leg) then %.2f s dwell. Setpoints at "
      "%.0f Hz.",
      waypoints_.size(), topic_source_ ? "jit/nav/local/mission" : waypoint_file_.c_str(),
      radius_m_, depth_radius_m_, dwell_s_, setpoint_rate_hz_);

    if (max_submerged_s_ > 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Submerged flag on nav/local/submerged below %.2f m from the entry depth. "
        "DEADMAN %.0f s: the RC link-loss failsafe is relaxed for at most that long in "
        "one stretch, then the mission ABORTs.",
        submerge_threshold_m_, max_submerged_s_);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "max_submerged_s is 0 - the submerged DEADMAN IS DISABLED. The RC link-loss "
        "failsafe can then be held off indefinitely by this node, and while it is held "
        "off the operator has no stop. Only crsf_channel_node's independent backstop "
        "timer still bounds it. Do not dive like this.");
    }
  }

private:
  enum class State { IDLE, RUNNING, COMPLETE, ABORTED };

  static const char * state_name(State s)
  {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::RUNNING: return "RUNNING";
      case State::COMPLETE: return "COMPLETE";
      case State::ABORTED: return "ABORTED";
    }
    return "?";
  }

  // ---- Waypoint file -------------------------------------------------------
  //
  // ROS 2 parameters cannot express a list of maps, so the mission lives in its
  // own plain YAML file loaded by path:
  //
  //   waypoints:
  //     - {x: 0.0, y: 0.0, z: -2.0}   # dive 2 m, no translation
  //     - {x: 2.0, y: 0.0}            # translate at -2.0 m - z is STICKY
  //     - {x: 2.0, y: 0.0, z:  0.0}   # rise back to the entry depth
  //
  // The omitted z on the second entry holds the depth commanded by the first,
  // NOT the entry depth. See z_offset_for().
  //
  void load_waypoints()
  {
    if (waypoint_file_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "No waypoint_file parameter set. No mission will run; the node will stay IDLE.");
      return;
    }
    try {
      const YAML::Node root = YAML::LoadFile(waypoint_file_);
      const YAML::Node list = root["waypoints"];
      if (!list || !list.IsSequence()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "'%s' has no 'waypoints:' sequence. No mission will run.", waypoint_file_.c_str());
        return;
      }
      for (const auto & item : list) {
        Waypoint wp;
        wp.x = item["x"] ? item["x"].as<double>() : 0.0;
        wp.y = item["y"] ? item["y"].as<double>() : 0.0;
        if (item["z"]) {
          wp.z = item["z"].as<double>();
          wp.has_z = true;
        }
        waypoints_.push_back(wp);
      }

      // Logged in a second pass so each line can show the effective depth that
      // z_offset_for() will resolve, which is what the vehicle actually flies -
      // not the raw field, which is absent on every inherited-depth leg.
      for (size_t i = 0; i < waypoints_.size(); ++i) {
        const double z = z_offset_for(i);
        RCLCPP_INFO(
          this->get_logger(), "  wp[%zu]  x=%+.2f  y=%+.2f  z=%+.2f  %s%s",
          i, waypoints_[i].x, waypoints_[i].y, z,
          waypoints_[i].has_z ? "(override)" : "(inherited)",
          leg_changes_depth(i) ? "  DEPTH LEG - vertical acceptance applies" : "");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to read waypoint file '%s': %s. No mission will run.",
        waypoint_file_.c_str(), e.what());
      waypoints_.clear();
    }
  }

  // ---- Topic mission -------------------------------------------------------
  //
  // Cache only. waypoints_ is replaced at mode entry, never here, so nothing
  // that arrives mid-mission can change the mission being flown.
  void mission_cb(const MissionXY::SharedPtr m)
  {
    cached_mission_.clear();
    for (const auto & w : m->waypoints) {
      Waypoint wp;
      wp.x = w.x;
      wp.y = w.y;
      // has_z stays false: MissionXY cannot carry a depth. See the header.
      cached_mission_.push_back(wp);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Topic mission cached: %zu waypoint(s) in %s. Takes effect at the next entry to "
      "LOCAL_GUIDED.", cached_mission_.size(), local_frame_.c_str());
    for (size_t i = 0; i < cached_mission_.size(); ++i) {
      RCLCPP_INFO(
        this->get_logger(), "  wp[%zu]  x=%+.2f  y=%+.2f  (absolute, entry depth)",
        i, cached_mission_[i].x, cached_mission_[i].y);
    }
  }

  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  // ---- Sticky depth --------------------------------------------------------
  //
  // The commanded depth of a leg, as an offset from entry_z: the last `z`
  // override at or before this index, or 0.0 (the entry depth) if none.
  //
  // Deliberately a pure function of the index rather than a member updated at
  // each advance. The mission's depth is then a property of the waypoint list
  // and cannot drift out of sync with index_ through a re-entry, an abort or a
  // mid-mission state change.
  double z_offset_for(size_t idx) const
  {
    double z = 0.0;
    for (size_t i = 0; i < waypoints_.size() && i <= idx; ++i) {
      if (waypoints_[i].has_z) {
        z = waypoints_[i].z;
      }
    }
    return z;
  }

  // True when this leg commands a depth change - the dive and rise legs. Only
  // these apply the vertical acceptance test; a level leg ignores depth error
  // exactly as this node always has. See the header.
  bool leg_changes_depth(size_t idx) const
  {
    const double prev = (idx == 0) ? 0.0 : z_offset_for(idx - 1);
    return std::fabs(z_offset_for(idx) - prev) > kDepthLegEpsilon;
  }

  // ---- Mode transitions ----------------------------------------------------
  void on_mode_entry()
  {
    // Snapshot: the topic mission in force for this whole run is the one cached
    // at this instant.
    if (topic_source_) {
      waypoints_ = cached_mission_;
    }
    if (waypoints_.empty()) {
      set_state(
        State::ABORTED,
        topic_source_ ? "no mission received on jit/nav/local/mission" : "no waypoints loaded");
      return;
    }
    if (!have_pose_ || !fresh(pose_stamp_, pose_timeout_s_)) {
      set_state(
        State::ABORTED,
        "no fresh /mavros/local_position/pose - refusing to start a mission against an "
        "unknown position");
      return;
    }

    datum_ = pose_.pose.position;
    entry_z_ = pose_.pose.position.z;
    index_ = 0;
    reached_flag_ = false;
    set_state(State::RUNNING, "datum captured");

    RCLCPP_INFO(
      this->get_logger(),
      "Mission start. Datum E=%.2f N=%.2f, entry depth z=%.2f. %zu waypoint(s), "
      "starting at index 0.",
      datum_.x, datum_.y, entry_z_, waypoints_.size());
  }

  void on_mode_exit()
  {
    if (state_ != State::IDLE) {
      set_state(State::IDLE, "left LOCAL_GUIDED");
    }
    index_ = 0;
    reached_flag_ = false;
  }

  void set_state(State s, const std::string & why)
  {
    if (s == state_) {
      return;
    }
    const State prev = state_;
    state_ = s;
    if (s == State::ABORTED) {
      RCLCPP_ERROR(
        this->get_logger(), "Mission %s -> ABORTED: %s", state_name(prev), why.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(), "Mission %s -> %s: %s", state_name(prev), state_name(s),
        why.c_str());
    }
  }

  // ---- Target construction -------------------------------------------------
  geometry_msgs::msg::PoseStamped current_target(const rclcpp::Time & now)
  {
    const Waypoint & wp = waypoints_[index_];

    geometry_msgs::msg::PoseStamped sp;
    sp.header.stamp = now;
    sp.header.frame_id = local_frame_;
    // A file waypoint is an offset from the datum; a topic waypoint is already
    // absolute in local_frame. Depth is the same rule for both.
    sp.pose.position.x = topic_source_ ? wp.x : datum_.x + wp.x;
    sp.pose.position.y = topic_source_ ? wp.y : datum_.y + wp.y;
    // Sticky: a leg with no z of its own holds the last commanded depth, not
    // the entry depth. See z_offset_for().
    sp.pose.position.z = entry_z_ + z_offset_for(index_);

    // Point along track: the bearing from where we are to the target.
    //
    // Recomputed every tick, but only the value present when
    // vehicle_interface_node forwards a target ever reaches ArduSub, and it
    // forwards on a position change alone. Position here is datum + offset, so
    // it is fixed for the whole leg and moves only at a waypoint advance - at
    // which moment the sub is sitting in the previous waypoint's acceptance
    // ball with the new target a full leg away, where the bearing is well
    // conditioned. The noisy close-in bearing is computed and then discarded.
    const double dx = sp.pose.position.x - pose_.pose.position.x;
    const double dy = sp.pose.position.y - pose_.pose.position.y;
    const double yaw = std::atan2(dy, dx);   // ENU: CCW from East
    sp.pose.orientation.x = 0.0;
    sp.pose.orientation.y = 0.0;
    sp.pose.orientation.z = std::sin(yaw * 0.5);
    sp.pose.orientation.w = std::cos(yaw * 0.5);
    return sp;
  }

  double horizontal_distance(const geometry_msgs::msg::PoseStamped & target) const
  {
    return std::hypot(
      target.pose.position.x - pose_.pose.position.x,
      target.pose.position.y - pose_.pose.position.y);
  }

  // Mirrors ArduPilot's verify_nav_wp(): latch on first touch, then run the
  // dwell clock without re-checking distance.
  //
  // The vertical term is the one addition. It applies only on a depth-changing
  // leg, because a dive waypoint sits directly below the datum and its
  // horizontal distance is ~0 before the vehicle has moved at all: a
  // horizontal-only test would latch "reached" on the first tick and complete
  // the mission without ever descending. On a level leg the depth error is
  // ignored as before, so a persistent surface trim error still cannot block
  // an acceptance.
  void check_reached(const rclcpp::Time & now, const geometry_msgs::msg::PoseStamped & target)
  {
    const double d = horizontal_distance(target);
    last_distance_ = d;

    if (!reached_flag_) {
      const double dz = target.pose.position.z - pose_.pose.position.z;
      const bool depth_leg = leg_changes_depth(index_);
      const bool depth_ok = !depth_leg || std::fabs(dz) <= depth_radius_m_;

      if (d <= radius_m_ && depth_ok) {
        reached_flag_ = true;
        reached_at_ = now;
        if (depth_leg) {
          RCLCPP_INFO(
            this->get_logger(),
            "Waypoint %zu reached (%.2f m <= %.2f m horizontal, %.2f m <= %.2f m "
            "vertical). Dwelling %.2f s.",
            index_, d, radius_m_, std::fabs(dz), depth_radius_m_, dwell_s_);
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "Waypoint %zu reached (%.2f m <= %.2f m horizontal; level leg, depth error "
            "%.2f m ignored). Dwelling %.2f s.",
            index_, d, radius_m_, std::fabs(dz), dwell_s_);
        }
      } else if (depth_leg && d <= radius_m_) {
        // Horizontally there but still descending or ascending. Worth seeing:
        // on a dive leg this is the normal state for the whole descent, and if
        // it never clears the vehicle cannot make depth.
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Waypoint %zu: horizontal ok, depth error %.2f m (need %.2f m). Still on the "
          "depth leg.", index_, std::fabs(dz), depth_radius_m_);
      }
      return;
    }

    if ((now - reached_at_).seconds() >= dwell_s_) {
      advance();
    }
  }

  void advance()
  {
    reached_flag_ = false;
    if (index_ + 1 < waypoints_.size()) {
      ++index_;
      RCLCPP_INFO(this->get_logger(), "Advancing to waypoint %zu.", index_);
      return;
    }
    // Last waypoint. Hold the final target so the vehicle station-keeps there;
    // system_monitor_node starts the hold_timeout_s clock that leads to
    // mission-safe.
    set_state(State::COMPLETE, "final waypoint reached; holding station");
  }

  // ---- The submerged flag --------------------------------------------------
  //
  // Decides what goes out on nav/local/submerged this tick, and runs the
  // deadman. See the header for what the flag does and why it is bounded.
  //
  // Only a live mission may assert it. IDLE, ABORTED and an inactive mode all
  // clear it, so a mission that stops for any reason also re-arms the RC
  // failsafe rather than leaving it relaxed.
  void update_submerged(const rclcpp::Time & now)
  {
    bool want = false;
    const char * why = "";

    if (state_ == State::RUNNING || state_ == State::COMPLETE) {
      // Commanded: asserted the instant a dive leg becomes current, which is
      // before the vehicle has descended and therefore before the antenna goes
      // under. That ordering is the point - the flag has to be live and fresh
      // at both consumers before the link actually drops.
      const bool cmd_deep = z_offset_for(index_) < -submerge_threshold_m_;

      // Measured: keeps the flag asserted through an ascent leg, which commands
      // a shallow depth while the vehicle is still deep. Without it the flag
      // would drop at the start of the ascent and the motor rail would be cut
      // halfway up. Requires a fresh pose - an unknown depth is not a reason to
      // keep the failsafe relaxed.
      const bool meas_deep = fresh(pose_stamp_, pose_timeout_s_) &&
        (pose_.pose.position.z - entry_z_) < -submerge_threshold_m_;

      want = cmd_deep || meas_deep;
      why = cmd_deep ? (meas_deep ? "commanded and measured" : "commanded") : "measured";
    }

    if (want) {
      if (!submerged_since_) {
        submerged_since_ = now;
        RCLCPP_WARN(
          this->get_logger(),
          "SUBMERGED (%s, threshold %.2f m) - asserting nav/local/submerged. The RC "
          "link-loss failsafe is now relaxed: if the link drops, the relay stays closed "
          "and the mode is held. There is no operator stop until this clears. Deadman "
          "%.0f s.",
          why, submerge_threshold_m_, max_submerged_s_);
      }

      const double held = (now - *submerged_since_).seconds();
      if (max_submerged_s_ > 0.0 && held >= max_submerged_s_) {
        want = false;
        RCLCPP_ERROR(
          this->get_logger(),
          "SUBMERGED DEADMAN EXPIRED after %.1f s (limit %.1f s). Dropping "
          "jit/nav/local/submerged and aborting: the RC link-loss failsafe re-arms, and if "
          "the link is still down the relay opens and the vehicle floats up.",
          held, max_submerged_s_);
        set_state(State::ABORTED, "submerged deadman expired");
      }
    }

    if (!want && submerged_since_) {
      RCLCPP_WARN(
        this->get_logger(),
        "No longer submerged after %.1f s - nav/local/submerged cleared, RC link-loss "
        "failsafe re-armed.", (now - *submerged_since_).seconds());
      submerged_since_.reset();
    }

    submerged_ = want;
  }

  // ---- Main loop -----------------------------------------------------------
  void tick()
  {
    const rclcpp::Time now = this->now();
    const bool mode_active =
      (granted_mode_ == Mode::LOCAL_GUIDED) && fresh(mode_stamp_, bus_timeout_s_);

    if (mode_active && !was_active_) {
      on_mode_entry();
    } else if (!mode_active && was_active_) {
      on_mode_exit();
    }
    was_active_ = mode_active;

    // Before the early returns, and before the setpoint goes out: this can
    // ABORT on the deadman, and when it does, the checks below must see the
    // new state so no further setpoint is published this tick.
    update_submerged(now);

    if (!mode_active || (state_ != State::RUNNING && state_ != State::COMPLETE)) {
      publish_outputs();
      return;
    }

    // A mission cannot continue without a position estimate.
    if (!fresh(pose_stamp_, pose_timeout_s_)) {
      set_state(State::ABORTED, "/mavros/local_position/pose went stale mid-mission");
      // Re-derive: the abort just happened, and an aborted mission must not
      // publish a submerged flag that keeps the RC failsafe relaxed for even
      // one more tick.
      update_submerged(now);
      publish_outputs();
      return;
    }

    const auto target = current_target(now);

    if (state_ == State::RUNNING) {
      check_reached(now, target);
    }

    // Republish continuously, but note this stream does NOT reach ArduSub at
    // this rate. vehicle_interface_node dedupes it and forwards a target only
    // when it moves, because each SET_POSITION_TARGET_LOCAL_NED restarts
    // AC_WPNav's s-curve and streaming them stops the vehicle accelerating.
    // What this rate is for is the cmd_timeout_s liveness gate over there.
    // Note this is NOT gated on vehicle/ready: vehicle_interface_node needs a
    // fresh setpoint to exist before it will arm, so waiting for ready here
    // would deadlock the two nodes against each other.
    setpoint_pub_->publish(target);
    publish_outputs();
  }

  // Both status topics, on every tick and every return path out of tick().
  // nav/local/submerged is publish-always by contract - see the header.
  void publish_outputs()
  {
    std_msgs::msg::String status;
    status.data = state_name(state_);
    status_pub_->publish(status);

    std_msgs::msg::Bool submerged;
    submerged.data = submerged_;
    submerged_pub_->publish(submerged);
  }

  // --- Config ---
  std::string waypoint_source_;
  bool topic_source_ {false};
  std::string waypoint_file_;
  std::string local_frame_;
  double radius_m_ {0.3};
  double dwell_s_ {1.0};
  double setpoint_rate_hz_ {10.0};
  double pose_timeout_s_ {1.0};
  double bus_timeout_s_ {0.5};
  double depth_radius_m_ {0.3};
  double submerge_threshold_m_ {0.2};
  double max_submerged_s_ {120.0};
  std::vector<Waypoint> waypoints_;      // the mission being flown
  std::vector<Waypoint> cached_mission_; // topic source: latest, applied at entry

  // --- Inputs ---
  geometry_msgs::msg::PoseStamped pose_;
  bool have_pose_ {false};
  rclcpp::Time pose_stamp_;
  uint8_t granted_mode_ {Mode::SAFE};
  rclcpp::Time mode_stamp_;

  // --- Mission state ---
  State state_ {State::IDLE};
  bool was_active_ {false};
  geometry_msgs::msg::Point datum_;
  double entry_z_ {0.0};
  size_t index_ {0};
  bool reached_flag_ {false};
  rclcpp::Time reached_at_;
  double last_distance_ {0.0};

  // --- Submerged flag ---
  // submerged_since_ is the deadman clock: set when the flag first asserts,
  // cleared when it drops. Not a duration counter - one continuous stretch.
  bool submerged_ {false};
  std::optional<rclcpp::Time> submerged_since_;

  // --- ROS interfaces ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr submerged_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<MissionXY>::SharedPtr mission_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalGuidedNode>());
  rclcpp::shutdown();
  return 0;
}
