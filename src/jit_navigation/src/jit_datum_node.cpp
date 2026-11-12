#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <GeographicLib/LocalCartesian.hpp>

using sensor_msgs::msg::NavSatFix;
using namespace std::placeholders;

class JitDatumBroadcaster : public rclcpp::Node {
public:
  JitDatumBroadcaster() : Node("jit_datum_broadcaster")
  {
    // Declare parameters
    wait_for_datum = this->declare_parameter<bool>("wait_for_datum", false);

    if (wait_for_datum) {
      // Use datum instead of waiting for a GPS fix to set the datum.
      // [latitude, longitude, heading] -- lat/lon in decimal degrees, heading in radians
      // (same convention as robot_localization's navsat_transform_node).
      datum = this->declare_parameter<std::vector<double>>("datum");
      if (datum[0] == 0.0 && datum[1] == 0.0 && datum[2] == 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Datum parameter is not set. Please set the datum parameter to a valid value.");
        rclcpp::shutdown();
        return;
      }

      // LocalCartesian's 3rd argument is the origin HEIGHT, not heading -- always 0.0 here.
      projection = std::make_unique<GeographicLib::LocalCartesian>(datum[0], datum[1], 0.0);
    }

    static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    junior_fix_sub = this->create_subscription<NavSatFix>("/junior/fix", rclcpp::SensorDataQoS(), std::bind(&JitDatumBroadcaster::onJuniorFix, this, _1));
    jit_fix_sub = this->create_subscription<NavSatFix>("/jit/fix", rclcpp::SensorDataQoS(), std::bind(&JitDatumBroadcaster::onJitFix, this, _1));
  }

private:
  std::unique_ptr<GeographicLib::LocalCartesian> projection;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
  rclcpp::Subscription<NavSatFix>::SharedPtr junior_fix_sub;
  rclcpp::Subscription<NavSatFix>::SharedPtr jit_fix_sub;

  bool published{false}, datum_set{false}, received_jit_fix{false}, wait_for_datum;
  std::vector<double> datum{0.0, 0.0, 0.0};  // [latitude, longitude, heading(rad)]
  NavSatFix jit_anchor_fix;  // Jit's *initial* fix, captured once -- never overwritten

  void onJuniorFix(const NavSatFix::UniquePtr msg);
  void onJitFix(const NavSatFix::UniquePtr msg);
  void publishAnchor();
};

void JitDatumBroadcaster::onJuniorFix(const NavSatFix::UniquePtr msg) {
  if (wait_for_datum || datum_set) {
    return;
  }

  if (!received_jit_fix) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Waiting for Jit's fix message before setting datum.");
    return;  // Wait for Jit's own initial fix message
  }

  datum[0] = msg->latitude;
  datum[1] = msg->longitude;
  RCLCPP_WARN(this->get_logger(), "Received reference fix. Setting the datum to the GPS fix (%.7f, %.7f).", datum[0], datum[1]);

  // Sanity-check the reference fix against Jit's initial position, in meters.
  GeographicLib::LocalCartesian candidate(datum[0], datum[1], 0.0);
  double cx, cy, cz;
  candidate.Forward(jit_anchor_fix.latitude, jit_anchor_fix.longitude, jit_anchor_fix.altitude, cx, cy, cz);
  double horizontal_dist = std::hypot(cx, cy);

  if (horizontal_dist < 0.1) {
    RCLCPP_WARN(
      this->get_logger(), "Reference GPS fix is very close to Jit's initial position.");
  }

  if (horizontal_dist > 100.0) {
    RCLCPP_WARN(
      this->get_logger(), "Reference GPS fix is more than 100 meters from Jit's initial position.");
  }

  // LocalCartesian's 3rd argument is the origin HEIGHT, not heading -- always 0.0 here.
  projection = std::make_unique<GeographicLib::LocalCartesian>(datum[0], datum[1], 0.0);

  datum_set = true;

  publishAnchor();
}

void JitDatumBroadcaster::onJitFix(const NavSatFix::UniquePtr msg) {
  if (received_jit_fix) {
    return;  // Already anchored off Jit's initial fix -- ignore the rest
  }

  jit_anchor_fix = *msg;
  received_jit_fix = true;

  if (!wait_for_datum) {
    // Fallback: no Junior fix has set the datum yet -- anchor to Jit's own
    // initial position.
    RCLCPP_WARN(this->get_logger(), "Setting the datum to Jit's initial position.");

    datum[0] = msg->latitude;
    datum[1] = msg->longitude;

    // LocalCartesian's 3rd argument is the origin HEIGHT, not heading -- always 0.0 here.
    projection = std::make_unique<GeographicLib::LocalCartesian>(datum[0], datum[1], 0.0);
  }

  if (!projection) {
    // wait_for_datum guarantees a projection from the constructor, and the
    // block above sets one otherwise -- but guard against it regardless.
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Jit's initial fix received but the datum/projection isn't ready yet; skipping anchor.");
    return;
  }

  publishAnchor();
}

void JitDatumBroadcaster::publishAnchor() {
  double x_enu, y_enu, z;
  projection->Forward(jit_anchor_fix.latitude, jit_anchor_fix.longitude, jit_anchor_fix.altitude, x_enu, y_enu, z);

  // Rotate the East-North-Up offset by the datum heading to express it in the
  // "map" frame
  double cos_h = std::cos(datum[2]);
  double sin_h = std::sin(datum[2]);
  double x = x_enu * cos_h - y_enu * sin_h;
  double y = x_enu * sin_h + y_enu * cos_h;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = jit_anchor_fix.header.stamp;
  t.header.frame_id = "map";
  t.child_frame_id = "jit/local_origin";
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  static_broadcaster->sendTransform(t);

  if (!published) {
    RCLCPP_INFO(
      this->get_logger(),
      "Anchored %s -> %s at Jit's initial GPS fix (%.7f, %.7f, %.2f) -> ENU offset "
      "(%.3f, %.3f, %.3f) m from datum",
      "map", "jit/local_origin",
      jit_anchor_fix.latitude, jit_anchor_fix.longitude, jit_anchor_fix.altitude, x, y, z);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Datum corrected by a later reference fix -- re-anchored %s -> %s "
      "at ENU offset (%.3f, %.3f, %.3f) m. Anything using the old offset "
      "may need to be re-evaluated.",
      "map", "jit/local_origin", x, y, z);
  }

  published = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JitDatumBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
