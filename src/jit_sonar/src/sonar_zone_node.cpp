// sonar_zone_node.cpp
//
// Post-processes the raw Ping360 echo: draws a cropped, colour-coded picture
// for rqt and (separately) gates the echo to a rectangular zone.
//
// ---------------------------------------------------------------------------
// WHAT IT SUBSCRIBES / PUBLISHES (relative names, so they follow the node's
// namespace - run it in the same namespace as ping360_node)
// ---------------------------------------------------------------------------
//   scan_echo  (ping360_sonar_msgs/SonarEcho)  in  - needs publish_echo: true
//   scan_view  (sensor_msgs/Image, rgb8)       out - cropped picture for rqt
//   scan_zone  (sensor_msgs/LaserScan)         out - zone-gated, once per sweep
//
// ---------------------------------------------------------------------------
// scan_view
// ---------------------------------------------------------------------------
// Drawn here, not taken from the driver's scan_image, so the field of view can
// be cropped to an exact radius (the driver's range_max is whole metres).
// Forward is up, left is left, like scan_image. Nothing is removed; the sweep
// is only COLOURED:
//
//   grey            the raw echo
//   red             walls   - long straight dense chains of returns
//   green           objects - clusters of returns that are not walls
//   light grey      returns that are neither (isolated noise)
//   yellow box      the zone used for scan_zone
//   dim blue rings  every foot
//
// ---------------------------------------------------------------------------
// HOW WALLS ARE TOLD FROM OBJECTS (no calibration)
// ---------------------------------------------------------------------------
// Once per completed sweep:
//   1. Take the nearest return >= intensity_threshold per angle, between
//      min_range and view_radius_m, as an x/y point.
//   2. Sequential RANSAC pulls out up to max_walls straight segments that are
//      at least wall_min_length_m long, wall_min_points dense, with no hole
//      over wall_max_gap_m. Those points are walls.
//   3. DBSCAN (cluster_eps_m, cluster_min_points) groups the remaining points;
//      each cluster is an object.
// Points on a wall's line also count as that wall, so an object's shadow on a
// wall does not leave a stub that looks like an object. See sonar_geometry.hpp.
// An object closer to a wall than wall_tolerance_m is absorbed into it; a flat
// object longer than wall_min_length_m looks like a wall.
//
// ---------------------------------------------------------------------------
// scan_zone
// ---------------------------------------------------------------------------
// A rectangle in the sonar frame: x forward, y left, metres. A bin at range r
// and angle a (after angle_offset_deg) is at (r cos a, r sin a); it counts
// only if inside the rectangle, at least min_range from the sonar, and
// brighter than intensity_threshold. The nearest such bin per angle is kept.
// Read from scan_echo (not scan) so an object in front of a wall is still
// found.
//
// Assumes a full 360 sweep with increasing angle, and angle_step_deg equal to
// the sonar's real step. The driver works in gradians and only accepts steps
// that divide 200: angle_step 2 gives exactly 2 gradians = 1.8 degrees.
// All parameters can be changed live with `ros2 param set`.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>

#include "sonar_geometry.hpp"

using ping360_sonar_msgs::msg::SonarEcho;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::LaserScan;
using sonar_geometry::Point;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kFoot = 0.3048;
constexpr float kNoReturn = std::numeric_limits<float>::infinity();

struct Rgb
{
  uint8_t r, g, b;
};
constexpr Rgb kWallColor{255, 40, 40};
constexpr Rgb kObjectColor{40, 255, 40};
constexpr Rgb kNoiseColor{200, 200, 200};
constexpr Rgb kZoneColor{255, 210, 0};
constexpr Rgb kRingColor{0, 60, 160};
constexpr Rgb kSonarColor{255, 255, 255};
}  // namespace

class SonarZoneNode : public rclcpp::Node
{
public:
  SonarZoneNode() : Node("sonar_zone_node")
  {
    // Ping360 blind zone: nothing closer than this is trustworthy.
    min_range_ = declare_parameter("min_range", 0.75);
    x_min_ = declare_parameter("zone_x_min", 0.80);
    x_max_ = declare_parameter("zone_x_max", 1.30);
    y_min_ = declare_parameter("zone_y_min", -0.30);
    y_max_ = declare_parameter("zone_y_max", 0.30);
    threshold_ = declare_parameter("intensity_threshold", 150);
    angle_offset_ = declare_parameter("angle_offset_deg", 0.0) * kPi / 180.0;
    // 2 gradians - what the sonar really does for angle_step: 2.
    step_ = declare_parameter("angle_step_deg", 1.8) * kPi / 180.0;
    view_radius_ = declare_parameter("view_radius_m", 1.5);
    mask_outside_zone_ = declare_parameter("mask_outside_zone", false);
    image_size_ = declare_parameter("image_size", 600);
    const int image_rate_ms{static_cast<int>(declare_parameter("image_rate_ms", 100))};

    walls_.tolerance = declare_parameter("wall_tolerance_m", 0.04);
    walls_.min_length = declare_parameter("wall_min_length_m", 0.50);
    walls_.min_points = static_cast<size_t>(declare_parameter("wall_min_points", 10));
    walls_.max_gap = declare_parameter("wall_max_gap_m", 0.30);
    walls_.max_walls = static_cast<int>(declare_parameter("max_walls", 4));
    walls_.iterations = static_cast<int>(declare_parameter("ransac_iterations", 300));
    cluster_eps_ = declare_parameter("cluster_eps_m", 0.15);
    cluster_min_points_ = static_cast<size_t>(declare_parameter("cluster_min_points", 3));

    configureSweep();
    configureImage();

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params)
      {
        for(const auto &p : params)
        {
          const auto &n{p.get_name()};
          if(n == "min_range") min_range_ = p.as_double();
          else if(n == "zone_x_min") x_min_ = p.as_double();
          else if(n == "zone_x_max") x_max_ = p.as_double();
          else if(n == "zone_y_min") y_min_ = p.as_double();
          else if(n == "zone_y_max") y_max_ = p.as_double();
          else if(n == "intensity_threshold") threshold_ = p.as_int();
          else if(n == "angle_offset_deg") angle_offset_ = p.as_double() * kPi / 180.0;
          else if(n == "view_radius_m") view_radius_ = p.as_double();
          else if(n == "mask_outside_zone") mask_outside_zone_ = p.as_bool();
          else if(n == "wall_tolerance_m") walls_.tolerance = p.as_double();
          else if(n == "wall_min_length_m") walls_.min_length = p.as_double();
          else if(n == "wall_min_points") walls_.min_points = static_cast<size_t>(p.as_int());
          else if(n == "wall_max_gap_m") walls_.max_gap = p.as_double();
          else if(n == "max_walls") walls_.max_walls = static_cast<int>(p.as_int());
          else if(n == "ransac_iterations") walls_.iterations = static_cast<int>(p.as_int());
          else if(n == "cluster_eps_m") cluster_eps_ = p.as_double();
          else if(n == "cluster_min_points") cluster_min_points_ = static_cast<size_t>(p.as_int());
          else if(n == "image_size")
          {
            image_size_ = p.as_int();
            configureImage();
          }
          else if(n == "angle_step_deg")
          {
            step_ = p.as_double() * kPi / 180.0;
            configureSweep();
          }
        }
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        return r;
      });

    pub_ = create_publisher<LaserScan>("scan_zone", rclcpp::SensorDataQoS());
    // Reliable so rqt_image_view (reliable by default) connects.
    view_pub_ = create_publisher<Image>("scan_view", rclcpp::QoS(1).reliable());
    view_timer_ = create_wall_timer(std::chrono::milliseconds(image_rate_ms),
                                    [this](){publishView();});
    sub_ = create_subscription<SonarEcho>(
      "scan_echo", rclcpp::SensorDataQoS(),
      [this](SonarEcho::ConstSharedPtr msg){onEcho(*msg);});

    RCLCPP_INFO(get_logger(), "View radius %.2f m, step %.2f deg, zone x[%.2f, %.2f] y[%.2f, %.2f] m",
                view_radius_, step_ * 180.0 / kPi, x_min_, x_max_, y_min_, y_max_);
  }

private:
  struct Object
  {
    Point centre;
    double radius;
    size_t points;
  };

  // The latest sweep's verdict, redrawn on every scan_view frame.
  struct Classification
  {
    std::vector<Point> noise, wall_points, object_points;
    std::vector<sonar_geometry::Wall> walls;
    std::vector<Object> objects;
  };

  void configureImage()
  {
    image_size_ = std::max(100, image_size_ & ~1);  // even, so the sonar is a pixel centre
    gray_.assign(static_cast<size_t>(image_size_) * image_size_, 0);
  }

  void configureSweep()
  {
    count_ = std::max<size_t>(1, static_cast<size_t>(std::lround(2 * kPi / step_)));
    zone_ranges_.assign(count_, kNoReturn);
    zone_intensities_.assign(count_, 0.f);
    near_ranges_.assign(count_, kNoReturn);
    last_index_ = 0;
  }

  // Paint this echo's wedge into gray_ by walking each sub-ray one pixel at a
  // time and looking up the bin, so there are no radial holes at long range_max.
  void paintEcho(const SonarEcho &echo)
  {
    const size_t bins{echo.intensities.size()};
    const double max_range{static_cast<double>(echo.range)};
    if(bins == 0 || max_range <= 0) return;

    const int half{image_size_ / 2};
    const double res{view_radius_ / half};  // metres per pixel
    const int rays{static_cast<int>(std::ceil(view_radius_ * step_ / res)) + 1};
    const double a0{echo.angle + angle_offset_};

    for(int k = 0; k < rays; k++)
    {
      const double a{a0 + step_ * (static_cast<double>(k) / rays - 0.5 + 0.5 / rays)};
      const double cx{std::cos(a)}, cy{std::sin(a)};
      for(int j = 1; j <= half; j++)
      {
        const double r{j * res};
        if(r > max_range) break;
        const long bin{std::lround(r * bins / max_range) - 1};
        if(bin < 0 || bin >= static_cast<long>(bins)) continue;
        const double x{r * cx}, y{r * cy};
        if(mask_outside_zone_ && !inZone(r, x, y)) continue;
        const int row{half - static_cast<int>(std::lround(x / res))};
        const int col{half - static_cast<int>(std::lround(y / res))};
        if(row < 0 || row >= image_size_ || col < 0 || col >= image_size_) continue;
        gray_[static_cast<size_t>(row) * image_size_ + col] = echo.intensities[bin];
      }
    }
  }

  bool inZone(double r, double x, double y) const
  {
    return r >= min_range_ && x >= x_min_ && x <= x_max_ && y >= y_min_ && y <= y_max_;
  }

  void onEcho(const SonarEcho &echo)
  {
    frame_id_ = echo.header.frame_id;
    paintEcho(echo);

    // Sonar angles run -pi..pi on a full sweep; +pi repeats -pi.
    const long raw{std::lround((echo.angle + kPi) / step_)};
    if(raw >= static_cast<long>(count_)) return;
    const auto index{static_cast<size_t>(std::max(0L, raw))};

    if(index < last_index_)
      endSweep(echo);
    last_index_ = index;

    const size_t bins{echo.intensities.size()};
    if(bins == 0) return;
    const double max_range{static_cast<double>(echo.range)};
    const double a{echo.angle + angle_offset_};
    const double cx{std::cos(a)}, cy{std::sin(a)};

    // Nearest bright bin overall (for the wall/object split) and nearest
    // bright bin inside the zone (for scan_zone).
    bool near_done{false}, zone_done{false};
    for(size_t i = 0; i < bins && !(near_done && zone_done); i++)
    {
      const double r{(i + 1) * max_range / bins};
      if(r < min_range_ || echo.intensities[i] < threshold_) continue;
      if(!near_done && r <= view_radius_)
      {
        near_ranges_[index] = static_cast<float>(r);
        near_done = true;
      }
      if(!zone_done && inZone(r, r * cx, r * cy))
      {
        zone_ranges_[index] = static_cast<float>(r);
        zone_intensities_[index] = echo.intensities[i] / 255.f;
        zone_done = true;
      }
    }
  }

  void endSweep(const SonarEcho &echo)
  {
    publishZoneScan(echo);
    classify();
    std::fill(zone_ranges_.begin(), zone_ranges_.end(), kNoReturn);
    std::fill(zone_intensities_.begin(), zone_intensities_.end(), 0.f);
    std::fill(near_ranges_.begin(), near_ranges_.end(), kNoReturn);
  }

  void publishZoneScan(const SonarEcho &echo)
  {
    LaserScan scan;
    scan.header = echo.header;
    scan.angle_min = static_cast<float>(-kPi);
    scan.angle_increment = static_cast<float>(step_);
    scan.angle_max = static_cast<float>(-kPi + step_ * (count_ - 1));
    scan.range_min = static_cast<float>(min_range_);
    scan.range_max = static_cast<float>(echo.range);
    scan.ranges = zone_ranges_;
    scan.intensities = zone_intensities_;
    pub_->publish(scan);
  }

  // Split the sweep's points into walls, objects and noise.
  void classify()
  {
    std::vector<Point> pts;
    for(size_t i = 0; i < count_; i++)
    {
      if(!std::isfinite(near_ranges_[i])) continue;
      const double a{-kPi + i * step_ + angle_offset_};
      pts.push_back({near_ranges_[i] * std::cos(a), near_ranges_[i] * std::sin(a)});
    }

    Classification next;
    next.walls = sonar_geometry::fitWalls(pts, walls_);

    std::vector<char> is_wall(pts.size(), 0);
    for(const auto &w : next.walls)
      for(auto i : w.members)
      {
        is_wall[i] = 1;
        next.wall_points.push_back(pts[i]);
      }

    // An object can shadow part of a wall, leaving a stub too short to be a
    // wall on its own. Anything lying on a wall's line is that wall.
    for(size_t i = 0; i < pts.size(); i++)
    {
      if(is_wall[i]) continue;
      for(const auto &w : next.walls)
      {
        const double dx{w.b.x - w.a.x}, dy{w.b.y - w.a.y}, len{std::hypot(dx, dy)};
        if(len <= 0) continue;
        const double off{(-(pts[i].x - w.a.x) * dy + (pts[i].y - w.a.y) * dx) / len};
        if(std::fabs(off) > walls_.tolerance) continue;
        is_wall[i] = 1;
        next.wall_points.push_back(pts[i]);
        break;
      }
    }

    std::vector<size_t> rest;
    for(size_t i = 0; i < pts.size(); i++)
      if(!is_wall[i]) rest.push_back(i);

    const auto label{sonar_geometry::dbscan(pts, rest, cluster_eps_, cluster_min_points_)};
    const int clusters{label.empty() ? 0 : *std::max_element(label.begin(), label.end()) + 1};
    std::vector<std::vector<size_t>> members(std::max(clusters, 0));
    for(size_t k = 0; k < rest.size(); k++)
    {
      if(label[k] < 0) next.noise.push_back(pts[rest[k]]);
      else members[label[k]].push_back(rest[k]);
    }
    for(const auto &m : members)
    {
      Point c{0, 0};
      for(auto i : m) { c.x += pts[i].x; c.y += pts[i].y; next.object_points.push_back(pts[i]); }
      c.x /= m.size(); c.y /= m.size();
      double radius{0};
      for(auto i : m) radius = std::max(radius, std::hypot(pts[i].x - c.x, pts[i].y - c.y));
      next.objects.push_back({c, radius + 0.03, m.size()});
    }

    RCLCPP_INFO(get_logger(), "Sweep: %zu points -> %zu wall(s) (%zu pts), %zu object(s), %zu noise",
                pts.size(), next.walls.size(), next.wall_points.size(),
                next.objects.size(), next.noise.size());
    for(const auto &o : next.objects)
      RCLCPP_INFO(get_logger(), "  object at (%.2f, %.2f) m, r %.2f m, %zu pts",
                  o.centre.x, o.centre.y, o.radius, o.points);
    result_ = std::move(next);
  }

  void publishView()
  {
    if(view_pub_->get_subscription_count() == 0) return;
    const int n{image_size_};
    const int half{n / 2};
    const double res{view_radius_ / half};

    Image img;
    img.header.stamp = now();
    img.header.frame_id = frame_id_;
    img.height = img.width = n;
    img.encoding = "rgb8";
    img.step = n * 3;
    img.data.resize(static_cast<size_t>(n) * n * 3);
    for(size_t i = 0; i < gray_.size(); i++)
      img.data[3 * i] = img.data[3 * i + 1] = img.data[3 * i + 2] = gray_[i];

    // (x, y) in metres, forward up and left left; `reach` makes a square dot.
    auto plot = [&](double x, double y, Rgb c, int reach = 0)
    {
      const int row0{half - static_cast<int>(std::lround(x / res))};
      const int col0{half - static_cast<int>(std::lround(y / res))};
      for(int row = row0 - reach; row <= row0 + reach; row++)
        for(int col = col0 - reach; col <= col0 + reach; col++)
        {
          if(row < 0 || row >= n || col < 0 || col >= n) continue;
          auto *px{&img.data[(static_cast<size_t>(row) * n + col) * 3]};
          px[0] = c.r; px[1] = c.g; px[2] = c.b;
        }
    };
    auto line = [&](Point a, Point b, Rgb c)
    {
      const int pts{std::max(2, static_cast<int>(std::hypot(b.x - a.x, b.y - a.y) / res))};
      for(int i = 0; i <= pts; i++)
      {
        const double t{static_cast<double>(i) / pts};
        plot(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), c);
      }
    };
    auto circle = [&](Point c, double radius, Rgb color)
    {
      const int pts{std::max(36, static_cast<int>(2 * kPi * radius / res))};
      for(int i = 0; i < pts; i++)
      {
        const double a{2 * kPi * i / pts};
        plot(c.x + radius * std::cos(a), c.y + radius * std::sin(a), color);
      }
    };

    for(int ring = 1; ring * kFoot <= view_radius_ + 1e-6; ring++)
      circle({0, 0}, ring * kFoot, kRingColor);

    const Point lo{x_min_, y_min_}, hi{x_max_, y_max_};
    line(lo, {hi.x, lo.y}, kZoneColor);
    line({hi.x, lo.y}, hi, kZoneColor);
    line(hi, {lo.x, hi.y}, kZoneColor);
    line({lo.x, hi.y}, lo, kZoneColor);

    for(const auto &p : result_.noise) plot(p.x, p.y, kNoiseColor, 1);
    for(const auto &w : result_.walls) line(w.a, w.b, kWallColor);
    for(const auto &p : result_.wall_points) plot(p.x, p.y, kWallColor, 3);
    for(const auto &p : result_.object_points) plot(p.x, p.y, kObjectColor, 3);
    for(const auto &o : result_.objects) circle(o.centre, o.radius, kObjectColor);

    plot(0, 0, kSonarColor, 2);  // the sonar

    view_pub_->publish(img);
  }

  double min_range_, x_min_, x_max_, y_min_, y_max_, angle_offset_, step_;
  double view_radius_, cluster_eps_;
  bool mask_outside_zone_;
  int threshold_, image_size_;
  size_t count_{}, last_index_{}, cluster_min_points_{};
  sonar_geometry::WallParams walls_{};
  std::string frame_id_;
  std::vector<uint8_t> gray_;
  std::vector<float> zone_ranges_, zone_intensities_, near_ranges_;
  Classification result_;

  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<Image>::SharedPtr view_pub_;
  rclcpp::TimerBase::SharedPtr view_timer_;
  rclcpp::Subscription<SonarEcho>::SharedPtr sub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarZoneNode>());
  rclcpp::shutdown();
  return 0;
}
