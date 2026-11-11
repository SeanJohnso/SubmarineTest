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
//        jit/task/active            latched, read by heartbeat_node
//        jit/task/state             latched, the current state's name
//
// This node holds no MAVROS client, publishes no cmd/* topic and is not on the
// health bus. local_guided_node flies every list, so acceptance, the submerged
// flag and the deadman stay in one place.
//
//   WAIT_FOR_LIST  local_guided holds position. Task inactive.
//   FOLLOW_PATH    the surface list from the other vessel. Task active from here.
//   PLAN_SURVEY    the end of the path has been reached. Settle, then generate.
//   SURVEY         dive, traverse, rise.
//   STABILIZE      surfaced, waiting to be steady at the final point.
//   DONE           holding station indefinitely. Task inactive.
//   FAILED         a list did not run to completion. Task inactive, no retry.
//
// Leaving AUTO from any state discards the list and returns to WAIT_FOR_LIST.
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "jit_msgs/msg/mission_xy.hpp"
#include "jit_msgs/msg/mode.hpp"
#include "jit_msgs/msg/task_mission.hpp"
#include "jit_msgs/msg/task_waypoint.hpp"

#include "jit_task/survey_plan.hpp"

using Mode = jit_msgs::msg::Mode;
using MissionXY = jit_msgs::msg::MissionXY;
using TaskMission = jit_msgs::msg::TaskMission;
using TaskWaypoint = jit_msgs::msg::TaskWaypoint;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class TaskNode : public rclcpp::Node
{
public:
  TaskNode()
  : Node("task_node")
  {
    survey_depth_m_ = this->declare_parameter<double>("survey_depth_m", 2.0);
    survey_length_m_ = this->declare_parameter<double>("survey_length_m", 10.0);
    max_survey_depth_m_ = this->declare_parameter<double>("max_survey_depth_m", 5.0);
    max_survey_length_m_ = this->declare_parameter<double>("max_survey_length_m", 30.0);
    settle_s_ = this->declare_parameter<double>("settle_s", 3.0);
    stabilize_radius_m_ = this->declare_parameter<double>("stabilize_radius_m", 0.5);
    stabilize_hold_s_ = this->declare_parameter<double>("stabilize_hold_s", 5.0);
    stabilize_timeout_s_ = this->declare_parameter<double>("stabilize_timeout_s", 30.0);
    start_timeout_s_ = this->declare_parameter<double>("start_timeout_s", 3.0);
    mode_timeout_s_ = this->declare_parameter<double>("mode_timeout_s", 0.5);
    status_timeout_s_ = this->declare_parameter<double>("status_timeout_s", 1.0);
    pose_timeout_s_ = this->declare_parameter<double>("pose_timeout_s", 1.0);
    submerged_timeout_s_ = this->declare_parameter<double>("submerged_timeout_s", 0.5);
    const double tick_hz = this->declare_parameter<double>("tick_hz", 10.0);

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    mode_stamp_ = status_stamp_ = pose_stamp_ = submerged_stamp_ = t0;
    auto_since_ = sent_at_ = handoff_at_ = stabilize_started_ = t0;

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    // Not latched - see TaskMission.msg.
    mission_pub_ = this->create_publisher<TaskMission>("jit/task/mission", 10);
    active_pub_ = this->create_publisher<std_msgs::msg::Bool>("jit/task/active", latched);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("jit/task/state", latched);

    list_sub_ = this->create_subscription<MissionXY>(
      "jit/nav/local/mission", latched,
      [this](const MissionXY::SharedPtr m) {list_ = m;});
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

    RCLCPP_INFO(
      this->get_logger(),
      "task_node up. Survey: dive %.2f m, traverse %.2f m along the final waypoint's heading.",
      survey_depth_m_, survey_length_m_);
    if (!survey_config_ok()) {
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

  // ---- States --------------------------------------------------------------
  void wait_for_list(const rclcpp::Time &)
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

    // Active the moment the list is taken, as the heartbeat should report.
    set_active(true);
    send_mission(frame_id_, waypoints);
    set_state(
      State::FOLLOW_PATH, "list received, " + std::to_string(waypoints.size()) + " waypoint(s)");
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
        handoff_at_ = now;
        set_state(State::PLAN_SURVEY, "end of path reached");
        return;
    }
  }

  void plan_survey(const rclcpp::Time & now)
  {
    if (status_ != "COMPLETE" || !fresh(status_stamp_, status_timeout_s_)) {
      fail("local_guided_node stopped holding the end of the path, status " + status_);
      return;
    }
    if ((now - handoff_at_).seconds() < settle_s_) {
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
  }

  // ---- Main loop -----------------------------------------------------------
  void tick()
  {
    const rclcpp::Time now = this->now();
    const bool in_auto = (mode_ == Mode::LOCAL_GUIDED) && fresh(mode_stamp_, mode_timeout_s_);

    if (in_auto && !was_in_auto_) {
      auto_since_ = now;
      RCLCPP_INFO(this->get_logger(), "In AUTO. Waiting for a waypoint list.");
    } else if (!in_auto && was_in_auto_) {
      list_.reset();
      stable_since_.reset();
      set_state(State::WAIT_FOR_LIST, "left AUTO");
      set_active(false);
    }
    was_in_auto_ = in_auto;

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
  double survey_depth_m_ {2.0};
  double survey_length_m_ {10.0};
  double max_survey_depth_m_ {5.0};
  double max_survey_length_m_ {30.0};
  double settle_s_ {3.0};
  double stabilize_radius_m_ {0.5};
  double stabilize_hold_s_ {5.0};
  double stabilize_timeout_s_ {30.0};
  double start_timeout_s_ {3.0};
  double mode_timeout_s_ {0.5};
  double status_timeout_s_ {1.0};
  double pose_timeout_s_ {1.0};
  double submerged_timeout_s_ {0.5};

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
  rclcpp::Time pose_stamp_;

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
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskNode>());
  rclcpp::shutdown();
  return 0;
}
