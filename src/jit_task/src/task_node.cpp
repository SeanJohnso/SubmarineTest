// task_node.cpp
//
// Sequences the pipeline survey task. Started only with waypoint_source:=task.
//
//   in   jit/nav/local/mission      the accepted list from waypoint_listener_node
//        jit/nav/local/status       local_guided_node's state
//        jit/nav/local/submerged    has the vehicle surfaced
//        jit/mode                   is the switch in AUTO
//        /mavros/local_position/pose   for the stabilized check
//   out  jit/task/mission           lists for local_guided_node to fly
//        jit/task/active            latched, read by heartbeat_node. Sent at
//                                   active_rate_hz in every state and mode, and
//                                   at once on every change
//        jit/task/state             latched, the current state's name
//
// This node holds no MAVROS client, publishes no cmd/* topic and is not on the
// health bus. local_guided_node flies every list, so acceptance, the submerged
// flag and the deadman stay in one place.
//
//   WAIT_FOR_LIST  local_guided holds position. Task inactive.
//   FOLLOW_PATH    the surface list, from the boat or the file. Task active from here.
//   PLAN_SURVEY    the end of the path has been reached. Pre-dive settle, then generate.
//                  With survey_enabled false this and SURVEY are skipped, and the
//                  end of the path goes straight to STABILIZE.
//   SURVEY         dive, traverse, rise.
//   STABILIZE      surfaced, waiting to be steady at the final point.
//   DONE           holding station indefinitely. Task inactive.
//   FAILED         a list did not run to completion. Task inactive, no retry.
//
// Leaving AUTO from any state discards the list and returns to WAIT_FOR_LIST.
//
// ---------------------------------------------------------------------------
// TWO SURFACE SOURCES, ONE END OF TASK
// ---------------------------------------------------------------------------
// The surface list only gets the sub to a point on the surface. Everything from
// the end of that list onward - pre-dive settle, survey, stabilize, DONE and
// the report - is the same code whichever source supplied it. list_source picks
// the source, once, at launch:
//
//   topic  jit/nav/local/mission from waypoint_listener_node (the boat). Starts
//          the moment a list stamped after this entry to AUTO arrives.
//   file   waypoint_file, the same file plain waypoint_source:=file flies. Read
//          once at startup. Starts start_settle_s after local_guided_node
//          reports HOLDING. x/y are offsets from the position held at that
//          moment, as in a plain file run.
//
// A file used here must be surface-only: any waypoint with a non-zero z rejects
// the whole file, because a list with depth in it is meant to be flown without
// the survey, by plain waypoint_source:=file. The final waypoint must carry yaw
// (degrees, ENU: 0 = East, counter-clockwise); it is the survey heading, the
// same role final_yaw plays in a boat list. yaw on any other waypoint is
// ignored. A rejected file never starts a run and never reports the task
// active; the error repeats in the log while the switch is in AUTO.
//
// ---------------------------------------------------------------------------
// THE REPORTED BUOY POSITION
// ---------------------------------------------------------------------------
// active_buoy_position in the report is the sub's GPS fix taken when it reached
// the last waypoint of the surface list, standing in for the first buoy. The
// sub has no buoy coordinates of its own, and it is sitting at that waypoint
// within GPS error. The fix is stored at the end of the path and sent in the
// report after surfacing. If no fresh fix existed then, the fix at report time
// is used instead, and failing that 0, 0.
//
// ---------------------------------------------------------------------------
// WHICH LISTS COUNT
// ---------------------------------------------------------------------------
// jit/nav/local/mission is latched, so an old list is delivered to this node
// when it starts. Only a list stamped after the switch entered AUTO is used. The
// listener stamps it on this same computer, so the two times are comparable. A
// restart of this node therefore waits for a new list rather than flying an old
// one.
//
// ---------------------------------------------------------------------------
// DETECTING THE END OF A LIST
// ---------------------------------------------------------------------------
// local_guided_node reports one status with no list identity, and it still
// reads COMPLETE from the previous list just after a new one is sent. So a list
// is complete only once RUNNING has been seen since it was sent, followed by
// COMPLETE. A list that never reports RUNNING within start_timeout_s has failed.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "jit_msgs/msg/mission_xy.hpp"
#include "jit_msgs/msg/mode.hpp"
#include "jit_msgs/msg/task_mission.hpp"
#include "jit_msgs/msg/task_waypoint.hpp"

#include "jit_task/survey_plan.hpp"

#include "robotx_comm/msg/pipe_line_segment_status.hpp"
#include "robotx_comm/msg/pipe_line_survey_report.hpp"

using Mode = jit_msgs::msg::Mode;
using MissionXY = jit_msgs::msg::MissionXY;
using TaskMission = jit_msgs::msg::TaskMission;
using TaskWaypoint = jit_msgs::msg::TaskWaypoint;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using SegmentStatus = robotx_comm::msg::PipeLineSegmentStatus;
using SurveyReport = robotx_comm::msg::PipeLineSurveyReport;

class TaskNode : public rclcpp::Node
{
public:
  TaskNode()
  : Node("task_node")
  {
    // Off by default, so diving always takes a deliberate enable_survey:=true.
    // Off, the task ends at the end of the surface path: stabilize, then hold.
    survey_enabled_ = this->declare_parameter<bool>("survey_enabled", false);
    survey_depth_m_ = this->declare_parameter<double>("survey_depth_m", 2.0);
    survey_length_m_ = this->declare_parameter<double>("survey_length_m", 10.0);
    max_survey_depth_m_ = this->declare_parameter<double>("max_survey_depth_m", 5.0);
    max_survey_length_m_ = this->declare_parameter<double>("max_survey_length_m", 30.0);
    // Hold at the end of the surface path for this long before diving.
    predive_settle_s_ = this->declare_parameter<double>("predive_settle_s", 3.0);

    // --- Surface source ---
    list_source_ = this->declare_parameter<std::string>("list_source", "topic");
    waypoint_file_ = this->declare_parameter<std::string>("waypoint_file", "");
    // The frame a file list is sent in. Must match local_guided_node's.
    local_frame_ = this->declare_parameter<std::string>("local_frame", "local_origin");
    // File source: how long local_guided_node must be HOLDING before the list
    // is sent, so the vehicle has armed and settled.
    start_settle_s_ = this->declare_parameter<double>("start_settle_s", 5.0);
    // How old the GPS fix may be when it is stored as the buoy position.
    gps_timeout_s_ = this->declare_parameter<double>("gps_timeout_s", 2.0);
    stabilize_radius_m_ = this->declare_parameter<double>("stabilize_radius_m", 0.5);
    stabilize_hold_s_ = this->declare_parameter<double>("stabilize_hold_s", 5.0);
    stabilize_timeout_s_ = this->declare_parameter<double>("stabilize_timeout_s", 30.0);
    start_timeout_s_ = this->declare_parameter<double>("start_timeout_s", 3.0);
    mode_timeout_s_ = this->declare_parameter<double>("mode_timeout_s", 0.5);
    status_timeout_s_ = this->declare_parameter<double>("status_timeout_s", 1.0);
    pose_timeout_s_ = this->declare_parameter<double>("pose_timeout_s", 1.0);
    submerged_timeout_s_ = this->declare_parameter<double>("submerged_timeout_s", 0.5);
    const double tick_hz = this->declare_parameter<double>("tick_hz", 10.0);
    // jit/task/active is republished at this rate whatever the state or mode,
    // so its arrival shows this node is alive. Changes still go out at once.
    active_rate_hz_ = this->declare_parameter<double>("active_rate_hz", 1.0);
    if (!(active_rate_hz_ > 0.0)) {
      RCLCPP_ERROR(
        this->get_logger(), "active_rate_hz must be > 0 (got %.2f). Using 1.0.", active_rate_hz_);
      active_rate_hz_ = 1.0;
    }

    // --- Pipeline survey report ---
    // Sent once per run, report_delay_s after the task ends in DONE or FAILED,
    // so the radio link has time to come back after surfacing.
    report_delay_s_ = this->declare_parameter<double>("report_delay_s", 15.0);
    report_topic_ = this->declare_parameter<std::string>(
      "report_topic", "/robotx_comm/jit/tx_pipeline");
    report_segments_ = this->declare_parameter<std::vector<int64_t>>(
      "report_segments", std::vector<int64_t>{1, 1, 2, 1});
    for (const auto v : report_segments_) {
      if (v < SegmentStatus::PIPELINE_SEGMENT_UNKNOWN ||
        v > SegmentStatus::PIPELINE_SEGMENT_DAMAGED)
      {
        RCLCPP_ERROR(
          this->get_logger(), "report_segments value %ld is not 0, 1 or 2 - it will be sent as 0.",
          static_cast<long>(v));
      }
    }

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    mode_stamp_ = status_stamp_ = pose_stamp_ = submerged_stamp_ = gps_stamp_ = t0;
    auto_since_ = sent_at_ = handoff_at_ = stabilize_started_ = t0;
    last_active_pub_ = t0;

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    // Not latched - see TaskMission.msg.
    mission_pub_ = this->create_publisher<TaskMission>("jit/task/mission", 10);
    active_pub_ = this->create_publisher<std_msgs::msg::Bool>("jit/task/active", latched);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("jit/task/state", latched);

    if (list_source_ == "file") {
      file_source_ = true;
      load_file();
    } else {
      if (list_source_ != "topic") {
        RCLCPP_ERROR(
          this->get_logger(), "Unknown list_source '%s' (expected 'topic' or 'file'). Using topic.",
          list_source_.c_str());
      }
      list_sub_ = this->create_subscription<MissionXY>(
        "jit/nav/local/mission", latched,
        [this](const MissionXY::SharedPtr m) {list_ = m;});
    }
    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {
        mode_ = m->mode;
        mode_stamp_ = this->now();
      });
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "jit/nav/local/status", 10,
      [this](const std_msgs::msg::String::SharedPtr m) {
        status_ = m->data;
        status_stamp_ = this->now();
        // Latched here rather than read in the tick, so that a sample between
        // two ticks cannot be missed.
        if (status_ == "RUNNING") {
          seen_running_ = true;
        }
      });
    submerged_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "jit/nav/local/submerged", 10,
      [this](const std_msgs::msg::Bool::SharedPtr m) {
        submerged_ = m->data;
        submerged_stamp_ = this->now();
      });
    // SensorDataQoS: MAVROS publishes BEST_EFFORT.
    // Reliable and latched, sent once: a receiver that reconnects after the
    // send still gets it. A reliable, latched publisher matches subscribers of
    // any reliability and durability.
    report_pub_ = this->create_publisher<SurveyReport>(report_topic_, latched);
    gps_sub_ = this->create_subscription<NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](const NavSatFix::SharedPtr m) {
        gps_ = m;
        gps_stamp_ = this->now();
      });

    pose_sub_ = this->create_subscription<PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const PoseStamped::SharedPtr m) {
        pose_ = m;
        pose_stamp_ = this->now();
      });

    publish_active();
    publish_state();

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / tick_hz)),
      std::bind(&TaskNode::tick, this));

    const std::string source_desc = file_source_ ?
      "file " + waypoint_file_ : std::string("topic (jit/nav/local/mission)");
    RCLCPP_INFO(this->get_logger(), "Surface list source: %s.", source_desc.c_str());

    if (!survey_enabled_) {
      RCLCPP_WARN(
        this->get_logger(),
        "task_node up. Survey DISABLED: the task follows the surface path, then holds. "
        "Launch with enable_survey:=true to dive.");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "task_node up. Survey: dive %.2f m, traverse %.2f m along the final waypoint's "
        "heading.", survey_depth_m_, survey_length_m_);
    }
    if (survey_enabled_ && !survey_config_ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Survey parameters are out of range (depth must be in (0, %.2f], length in (0, %.2f]). "
        "The surface path will still run, but the task will FAIL before diving.",
        max_survey_depth_m_, max_survey_length_m_);
    }
  }

private:
  enum class State
  {
    WAIT_FOR_LIST, FOLLOW_PATH, PLAN_SURVEY, SURVEY, STABILIZE, DONE, FAILED
  };

  enum class Leg { IN_PROGRESS, COMPLETE, FAILED };

  static const char * state_name(State s)
  {
    switch (s) {
      case State::WAIT_FOR_LIST: return "WAIT_FOR_LIST";
      case State::FOLLOW_PATH: return "FOLLOW_PATH";
      case State::PLAN_SURVEY: return "PLAN_SURVEY";
      case State::SURVEY: return "SURVEY";
      case State::STABILIZE: return "STABILIZE";
      case State::DONE: return "DONE";
      case State::FAILED: return "FAILED";
    }
    return "?";
  }

  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  bool survey_config_ok() const
  {
    return std::isfinite(survey_depth_m_) && survey_depth_m_ > 0.0 &&
           survey_depth_m_ <= max_survey_depth_m_ &&
           std::isfinite(survey_length_m_) && survey_length_m_ > 0.0 &&
           survey_length_m_ <= max_survey_length_m_;
  }

  void publish_active()
  {
    std_msgs::msg::Bool m;
    m.data = active_;
    active_pub_->publish(m);
    last_active_pub_ = this->now();
  }

  void publish_state()
  {
    std_msgs::msg::String m;
    m.data = state_name(state_);
    state_pub_->publish(m);
  }

  void set_active(bool active)
  {
    if (active != active_) {
      active_ = active;
      publish_active();
    }
  }

  void set_state(State s, const std::string & why)
  {
    if (s == state_) {
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "Task %s -> %s: %s", state_name(state_), state_name(s), why.c_str());
    state_ = s;
    publish_state();
  }

  void fail(const std::string & why)
  {
    RCLCPP_ERROR(this->get_logger(), "Task FAILED: %s", why.c_str());
    set_state(State::FAILED, why);
    set_active(false);
    schedule_report();
  }

  // ---- Pipeline survey report ----------------------------------------------
  void schedule_report()
  {
    if (report_sent_ || report_due_) {
      return;
    }
    report_due_ = this->now() + rclcpp::Duration::from_seconds(report_delay_s_);
    RCLCPP_INFO(
      this->get_logger(), "Pipeline report scheduled in %.0f s on %s.", report_delay_s_,
      report_topic_.c_str());
  }

  // Runs whatever the mode, so leaving AUTO after the task ends does not stop
  // the report.
  void service_report(const rclcpp::Time & now)
  {
    if (!report_due_ || now < *report_due_) {
      return;
    }
    report_due_.reset();
    report_sent_ = true;

    SurveyReport r;
    r.time = now;
    // The fix stored at the end of the surface path, standing in for the
    // buoy's position. See the header.
    if (buoy_fix_) {
      r.active_buoy_position.latitude = buoy_fix_->first;
      r.active_buoy_position.longitude = buoy_fix_->second;
    } else if (gps_) {
      RCLCPP_WARN(
        this->get_logger(),
        "No GPS fix was stored at the end of the surface path - reporting the current "
        "position instead.");
      r.active_buoy_position.latitude = gps_->latitude;
      r.active_buoy_position.longitude = gps_->longitude;
    } else {
      RCLCPP_WARN(
        this->get_logger(), "No /mavros/global_position/global yet - reporting position 0, 0.");
    }
    std::string seq;
    for (const auto v : report_segments_) {
      uint8_t value = SegmentStatus::PIPELINE_SEGMENT_UNKNOWN;
      if (v == SegmentStatus::PIPELINE_SEGMENT_INTACT) {
        value = SegmentStatus::PIPELINE_SEGMENT_INTACT;
      } else if (v == SegmentStatus::PIPELINE_SEGMENT_DAMAGED) {
        value = SegmentStatus::PIPELINE_SEGMENT_DAMAGED;
      }
      // PipeLineSegmentStatus declares only constants, so a segment cannot be
      // pushed as a bare constant: segments holds messages. rosidl gives a
      // constants-only message one generated uint8 member, and the constant
      // goes there. A C++ receiver reads it from the same member; Python's
      // generated class hides it.
      SegmentStatus s;
      s.structure_needs_at_least_one_member = value;
      r.segments.push_back(s);
      seq += std::to_string(value) + " ";
    }
    report_pub_->publish(r);
    RCLCPP_INFO(
      this->get_logger(), "PIPELINE REPORT sent on %s: segments [ %s], position %.7f, %.7f.",
      report_topic_.c_str(), seq.c_str(), r.active_buoy_position.latitude,
      r.active_buoy_position.longitude);
  }

  void send_mission(const std::string & frame_id, const std::vector<TaskWaypoint> & waypoints)
  {
    TaskMission m;
    m.header.stamp = this->now();
    m.header.frame_id = frame_id;
    m.waypoints = waypoints;
    seen_running_ = false;
    sent_at_ = this->now();
    mission_pub_->publish(m);
  }

  // Where the list most recently sent has got to. See the header.
  Leg leg_progress(const rclcpp::Time & now, std::string & why) const
  {
    if (!fresh(status_stamp_, status_timeout_s_)) {
      why = "jit/nav/local/status went stale";
      return Leg::FAILED;
    }
    if (!seen_running_) {
      if (status_ == "ABORTED") {
        why = "local_guided_node aborted before the list started";
        return Leg::FAILED;
      }
      if ((now - sent_at_).seconds() >= start_timeout_s_) {
        why = "local_guided_node did not start the list";
        return Leg::FAILED;
      }
      return Leg::IN_PROGRESS;
    }
    if (status_ == "RUNNING") {
      return Leg::IN_PROGRESS;
    }
    if (status_ == "COMPLETE") {
      return Leg::COMPLETE;
    }
    why = "local_guided_node left the list, status " + status_;
    return Leg::FAILED;
  }

  // ---- File source ---------------------------------------------------------
  //
  // Reads and checks waypoint_file once. Any problem leaves file_error_ set,
  // and wait_for_file() then refuses to start. See the header for the rules.
  void load_file()
  {
    file_offsets_.clear();
    file_error_.clear();
    if (waypoint_file_.empty()) {
      file_error_ = "no waypoint_file parameter set";
    } else {
      try {
        const YAML::Node root = YAML::LoadFile(waypoint_file_);
        const YAML::Node list = root["waypoints"];
        if (!list || !list.IsSequence() || list.size() == 0) {
          file_error_ = "no 'waypoints:' entries";
        }
        for (size_t i = 0; file_error_.empty() && i < list.size(); ++i) {
          const YAML::Node item = list[i];
          const double x = item["x"] ? item["x"].as<double>() : 0.0;
          const double y = item["y"] ? item["y"].as<double>() : 0.0;
          const double z = item["z"] ? item["z"].as<double>() : 0.0;
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            file_error_ = "waypoint " + std::to_string(i) + " has a non-finite value";
          } else if (z != 0.0) {
            file_error_ = "waypoint " + std::to_string(i) + " has z = " + std::to_string(z) +
              " - a task surface list must be surface-only. Fly lists with depth "
              "using waypoint_source:=file";
          }
          file_offsets_.emplace_back(x, y);
        }
        if (file_error_.empty()) {
          const YAML::Node last = list[list.size() - 1];
          if (!last["yaw"]) {
            file_error_ = "the final waypoint has no yaw - it is the survey heading";
          } else {
            const double yaw_deg = last["yaw"].as<double>();
            if (!std::isfinite(yaw_deg)) {
              file_error_ = "the final waypoint's yaw is not finite";
            } else {
              file_final_yaw_ = yaw_deg * M_PI / 180.0;
            }
          }
        }
      } catch (const std::exception & e) {
        file_error_ = std::string("could not read it: ") + e.what();
      }
    }

    if (!file_error_.empty()) {
      file_offsets_.clear();
      RCLCPP_ERROR(
        this->get_logger(), "Waypoint file '%s' REJECTED: %s. The task will not start.",
        waypoint_file_.c_str(), file_error_.c_str());
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint file '%s' accepted: %zu surface waypoint(s), survey heading %.1f deg ENU. "
      "Offsets are from the position held when the run starts.",
      waypoint_file_.c_str(), file_offsets_.size(), file_final_yaw_ * 180.0 / M_PI);
    for (size_t i = 0; i < file_offsets_.size(); ++i) {
      RCLCPP_INFO(
        this->get_logger(), "  wp[%zu]  dx=%+.2f  dy=%+.2f", i,
        file_offsets_[i].first, file_offsets_[i].second);
    }
  }

  // ---- States --------------------------------------------------------------
  void wait_for_list(const rclcpp::Time & now)
  {
    if (file_source_) {
      wait_for_file(now);
    } else {
      wait_for_topic_list();
    }
  }

  // The start shared by both sources. waypoints are absolute in frame_id_;
  // path_end_* and final_yaw_ are already set.
  void start_surface_path(const std::vector<TaskWaypoint> & waypoints)
  {
    // Active the moment the run starts, as the heartbeat should report.
    set_active(true);
    // A new run: it gets its own report and its own buoy fix.
    report_due_.reset();
    report_sent_ = false;
    buoy_fix_.reset();
    send_mission(frame_id_, waypoints);
    set_state(
      State::FOLLOW_PATH,
      std::string(file_source_ ? "file" : "list") + " received, " +
      std::to_string(waypoints.size()) + " waypoint(s)");
  }

  void wait_for_file(const rclcpp::Time & now)
  {
    if (!file_error_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "In AUTO, but the waypoint file was rejected (%s). Not starting.", file_error_.c_str());
      return;
    }
    if (status_ != "HOLDING" || !fresh(status_stamp_, status_timeout_s_)) {
      holding_since_.reset();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for local_guided_node to hold (status '%s').", status_.c_str());
      return;
    }
    if (!holding_since_) {
      holding_since_ = now;
      RCLCPP_INFO(
        this->get_logger(), "local_guided_node holding. Starting the file list in %.1f s.",
        start_settle_s_);
    }
    if ((now - *holding_since_).seconds() < start_settle_s_) {
      return;
    }
    if (!pose_ || !fresh(pose_stamp_, pose_timeout_s_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Settled, but no fresh /mavros/local_position/pose to offset the file from.");
      return;
    }

    // The datum is where the sub is holding now, as in a plain file run.
    const double datum_x = pose_->pose.position.x;
    const double datum_y = pose_->pose.position.y;
    frame_id_ = local_frame_;
    std::vector<TaskWaypoint> waypoints;
    for (const auto & o : file_offsets_) {
      TaskWaypoint t;
      t.x = datum_x + o.first;
      t.y = datum_y + o.second;
      t.has_z = false;
      waypoints.push_back(t);
    }
    path_end_x_ = waypoints.back().x;
    path_end_y_ = waypoints.back().y;
    final_yaw_ = file_final_yaw_;

    RCLCPP_INFO(
      this->get_logger(), "File list start. Datum x=%+.2f y=%+.2f in %s.", datum_x, datum_y,
      frame_id_.c_str());
    start_surface_path(waypoints);
  }

  void wait_for_topic_list()
  {
    if (!list_) {
      return;
    }
    const rclcpp::Time stamp(list_->header.stamp, this->get_clock()->get_clock_type());
    if (stamp < auto_since_) {
      RCLCPP_INFO(this->get_logger(), "Ignoring a list from before this entry to AUTO.");
      list_.reset();
      return;
    }
    if (list_->waypoints.empty() || !std::isfinite(list_->final_yaw)) {
      RCLCPP_ERROR(this->get_logger(), "Ignoring a list with no waypoints or no final heading.");
      list_.reset();
      return;
    }
    if (status_ != "HOLDING" || !fresh(status_stamp_, status_timeout_s_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Have a list, but local_guided_node is not HOLDING (status '%s').", status_.c_str());
      return;
    }

    frame_id_ = list_->header.frame_id;
    path_end_x_ = list_->waypoints.back().x;
    path_end_y_ = list_->waypoints.back().y;
    final_yaw_ = list_->final_yaw;

    std::vector<TaskWaypoint> waypoints;
    for (const auto & w : list_->waypoints) {
      TaskWaypoint t;
      t.x = w.x;
      t.y = w.y;
      t.has_z = false;
      waypoints.push_back(t);
    }

    start_surface_path(waypoints);
  }

  void follow_path(const rclcpp::Time & now)
  {
    std::string why;
    switch (leg_progress(now, why)) {
      case Leg::IN_PROGRESS:
        return;
      case Leg::FAILED:
        fail(why);
        return;
      case Leg::COMPLETE:
        RCLCPP_INFO(
          this->get_logger(), "END OF PATH reached at x=%+.2f y=%+.2f, heading %.1f deg ENU.",
          path_end_x_, path_end_y_, final_yaw_ * 180.0 / M_PI);
        store_buoy_fix();
        if (!survey_enabled_) {
          // Surface-only run: the end of the path is the final point.
          final_x_ = path_end_x_;
          final_y_ = path_end_y_;
          stabilize_started_ = now;
          stable_since_.reset();
          set_state(State::STABILIZE, "end of path reached, survey disabled");
          return;
        }
        handoff_at_ = now;
        set_state(State::PLAN_SURVEY, "end of path reached");
        return;
    }
  }

  // The sub is at the last surface waypoint, standing in for the first buoy.
  void store_buoy_fix()
  {
    if (!gps_ || !fresh(gps_stamp_, gps_timeout_s_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "No fresh GPS fix at the end of the path - the report will use the position at "
        "report time.");
      return;
    }
    buoy_fix_ = std::make_pair(gps_->latitude, gps_->longitude);
    RCLCPP_INFO(
      this->get_logger(), "Buoy position stored for the report: %.7f, %.7f.",
      buoy_fix_->first, buoy_fix_->second);
  }

  void plan_survey(const rclcpp::Time & now)
  {
    if (status_ != "COMPLETE" || !fresh(status_stamp_, status_timeout_s_)) {
      fail("local_guided_node stopped holding the end of the path, status " + status_);
      return;
    }
    if ((now - handoff_at_).seconds() < predive_settle_s_) {
      return;
    }
    if (!survey_config_ok()) {
      fail("survey parameters are out of range");
      return;
    }

    const auto legs = jit_task::make_survey(
      path_end_x_, path_end_y_, final_yaw_, survey_depth_m_, survey_length_m_);

    std::vector<TaskWaypoint> waypoints;
    for (const auto & leg : legs) {
      TaskWaypoint t;
      t.x = leg.x;
      t.y = leg.y;
      t.has_z = leg.has_z;
      t.z = leg.z;
      waypoints.push_back(t);
    }
    final_x_ = legs.back().x;
    final_y_ = legs.back().y;

    send_mission(frame_id_, waypoints);
    set_state(State::SURVEY, "survey sent");
  }

  void survey(const rclcpp::Time & now)
  {
    std::string why;
    switch (leg_progress(now, why)) {
      case Leg::IN_PROGRESS:
        return;
      case Leg::FAILED:
        fail(why);
        return;
      case Leg::COMPLETE:
        stabilize_started_ = now;
        stable_since_.reset();
        set_state(State::STABILIZE, "survey complete, surfaced");
        return;
    }
  }

  void stabilize(const rclcpp::Time & now)
  {
    if (status_ != "COMPLETE" || !fresh(status_stamp_, status_timeout_s_)) {
      fail("local_guided_node stopped holding the final point, status " + status_);
      return;
    }

    const bool surfaced = fresh(submerged_stamp_, submerged_timeout_s_) && !submerged_;
    const bool in_place = pose_ && fresh(pose_stamp_, pose_timeout_s_) &&
      std::hypot(
      pose_->pose.position.x - final_x_,
      pose_->pose.position.y - final_y_) <= stabilize_radius_m_;

    if (surfaced && in_place) {
      if (!stable_since_) {
        stable_since_ = now;
      }
      if ((now - *stable_since_).seconds() >= stabilize_hold_s_) {
        finish("stabilized at the final point");
        return;
      }
    } else {
      stable_since_.reset();
    }

    // The heartbeat must not stick on an active task. This only changes what is
    // reported: the vehicle goes on holding the final point either way.
    if ((now - stabilize_started_).seconds() >= stabilize_timeout_s_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Not stable after %.0f s (surfaced: %s, in place: %s). Reporting the task finished.",
        stabilize_timeout_s_, surfaced ? "yes" : "no", in_place ? "yes" : "no");
      finish("stabilize timeout");
    }
  }

  void finish(const std::string & why)
  {
    set_state(State::DONE, why);
    set_active(false);
    schedule_report();
  }

  // ---- Main loop -----------------------------------------------------------
  void tick()
  {
    const rclcpp::Time now = this->now();
    const bool in_auto = (mode_ == Mode::LOCAL_GUIDED) && fresh(mode_stamp_, mode_timeout_s_);

    // Before anything that can return early: this runs in every state and mode.
    if ((now - last_active_pub_).seconds() >= 1.0 / active_rate_hz_) {
      publish_active();
    }

    if (in_auto && !was_in_auto_) {
      auto_since_ = now;
      RCLCPP_INFO(this->get_logger(), "In AUTO. Waiting for a waypoint list.");
    } else if (!in_auto && was_in_auto_) {
      list_.reset();
      stable_since_.reset();
      holding_since_.reset();
      set_state(State::WAIT_FOR_LIST, "left AUTO");
      set_active(false);
    }
    was_in_auto_ = in_auto;

    service_report(now);

    if (!in_auto) {
      return;
    }

    switch (state_) {
      case State::WAIT_FOR_LIST: wait_for_list(now); break;
      case State::FOLLOW_PATH: follow_path(now); break;
      case State::PLAN_SURVEY: plan_survey(now); break;
      case State::SURVEY: survey(now); break;
      case State::STABILIZE: stabilize(now); break;
      case State::DONE: break;
      case State::FAILED: break;
    }
  }

  // --- Config ---
  bool survey_enabled_ {false};
  double survey_depth_m_ {2.0};
  double survey_length_m_ {10.0};
  double max_survey_depth_m_ {5.0};
  double max_survey_length_m_ {30.0};
  double predive_settle_s_ {3.0};
  double start_settle_s_ {5.0};
  double gps_timeout_s_ {2.0};
  std::string list_source_;
  std::string waypoint_file_;
  std::string local_frame_;
  bool file_source_ {false};
  double stabilize_radius_m_ {0.5};
  double stabilize_hold_s_ {5.0};
  double stabilize_timeout_s_ {30.0};
  double start_timeout_s_ {3.0};
  double mode_timeout_s_ {0.5};
  double status_timeout_s_ {1.0};
  double pose_timeout_s_ {1.0};
  double submerged_timeout_s_ {0.5};
  double report_delay_s_ {15.0};
  double active_rate_hz_ {1.0};
  std::string report_topic_;
  std::vector<int64_t> report_segments_;

  // --- Inputs ---
  MissionXY::SharedPtr list_;
  uint8_t mode_ {Mode::SAFE};
  rclcpp::Time mode_stamp_;
  std::string status_;
  rclcpp::Time status_stamp_;
  bool seen_running_ {false};
  bool submerged_ {false};
  rclcpp::Time submerged_stamp_;
  PoseStamped::SharedPtr pose_;
  NavSatFix::SharedPtr gps_;
  std::optional<rclcpp::Time> report_due_;
  bool report_sent_ {false};
  rclcpp::Time last_active_pub_;
  rclcpp::Time pose_stamp_;
  rclcpp::Time gps_stamp_;
  // GPS fix stored at the end of the surface path, for the report.
  std::optional<std::pair<double, double>> buoy_fix_;
  // File source: the checked offsets, the survey heading in radians, and why
  // the file was rejected (empty when accepted).
  std::vector<std::pair<double, double>> file_offsets_;
  double file_final_yaw_ {0.0};
  std::string file_error_;
  // File source: when local_guided_node was first seen HOLDING this wait.
  std::optional<rclcpp::Time> holding_since_;

  // --- Task state ---
  State state_ {State::WAIT_FOR_LIST};
  bool active_ {false};
  bool was_in_auto_ {false};
  rclcpp::Time auto_since_;
  rclcpp::Time sent_at_;
  rclcpp::Time handoff_at_;
  rclcpp::Time stabilize_started_;
  std::optional<rclcpp::Time> stable_since_;
  std::string frame_id_;
  double path_end_x_ {0.0};
  double path_end_y_ {0.0};
  double final_yaw_ {0.0};
  double final_x_ {0.0};
  double final_y_ {0.0};

  // --- ROS interfaces ---
  rclcpp::Publisher<TaskMission>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<MissionXY>::SharedPtr list_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr submerged_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<SurveyReport>::SharedPtr report_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskNode>());
  rclcpp::shutdown();
  return 0;
}
