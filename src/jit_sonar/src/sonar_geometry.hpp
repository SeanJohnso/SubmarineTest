// sonar_geometry.hpp
//
// Calibration-free separation of walls from objects in a 2-D sonar point set.
//
//   fitWalls  - sequential RANSAC for long, dense, straight segments (walls)
//   dbscan    - density clustering of whatever is left (objects)
//
// A "wall" is a chain of points lying within `tolerance` of one line, with
// neighbours no more than `max_gap` apart along it, at least `min_length` long
// and `min_points` strong. Anything shorter - a bucket, a pole - fails the
// length test and stays an object candidate.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <utility>
#include <vector>

namespace sonar_geometry
{

struct Point
{
  double x, y;
};

struct WallParams
{
  double tolerance;     // m, max distance from the line
  double min_length;    // m, shortest segment that counts as a wall
  size_t min_points;
  double max_gap;       // m, largest hole inside one wall
  int max_walls;
  int iterations;       // RANSAC hypotheses per wall
};

struct Wall
{
  Point a, b;                    // segment ends
  std::vector<size_t> members;   // indices into the point set
};

namespace detail
{

struct Line
{
  Point p;
  double dx, dy;  // unit direction
};

struct Run
{
  double t0 = 0, t1 = 0;
  std::vector<size_t> members;
};

// Longest chain (by point count) of available points within `tol` of `line`
// whose neighbours along the line are at most `max_gap` apart.
inline Run longestRun(const std::vector<Point> &pts, const std::vector<char> &available,
                      const Line &line, double tol, double max_gap)
{
  std::vector<std::pair<double, size_t>> on;
  for(size_t i = 0; i < pts.size(); i++)
  {
    if(!available[i]) continue;
    const double rx{pts[i].x - line.p.x}, ry{pts[i].y - line.p.y};
    if(std::fabs(-rx * line.dy + ry * line.dx) <= tol)
      on.emplace_back(rx * line.dx + ry * line.dy, i);
  }
  std::sort(on.begin(), on.end());

  Run best;
  size_t start{0};
  for(size_t e = 1; e <= on.size(); e++)
  {
    if(e == on.size() || on[e].first - on[e - 1].first > max_gap)
    {
      if(e - start > best.members.size())
      {
        best.t0 = on[start].first;
        best.t1 = on[e - 1].first;
        best.members.clear();
        for(size_t k = start; k < e; k++) best.members.push_back(on[k].second);
      }
      start = e;
    }
  }
  return best;
}

// Total-least-squares line through the given points.
inline Line refit(const std::vector<Point> &pts, const std::vector<size_t> &idx)
{
  double mx{}, my{};
  for(auto i : idx) { mx += pts[i].x; my += pts[i].y; }
  mx /= idx.size(); my /= idx.size();
  double sxx{}, sxy{}, syy{};
  for(auto i : idx)
  {
    const double dx{pts[i].x - mx}, dy{pts[i].y - my};
    sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
  }
  const double theta{0.5 * std::atan2(2 * sxy, sxx - syy)};
  return {{mx, my}, std::cos(theta), std::sin(theta)};
}

inline bool isWall(const Run &run, const WallParams &prm)
{
  return run.members.size() >= prm.min_points && run.t1 - run.t0 >= prm.min_length;
}

}  // namespace detail

inline std::vector<Wall> fitWalls(const std::vector<Point> &pts, const WallParams &prm)
{
  using namespace detail;
  std::vector<Wall> walls;
  std::vector<char> available(pts.size(), 1);
  std::mt19937 rng(12345);  // fixed seed: same sweep, same answer

  for(int w = 0; w < prm.max_walls; w++)
  {
    std::vector<size_t> pool;
    for(size_t i = 0; i < pts.size(); i++)
      if(available[i]) pool.push_back(i);
    if(pool.size() < prm.min_points) break;

    std::uniform_int_distribution<size_t> pick(0, pool.size() - 1);
    Run best;
    Line best_line{};
    for(int it = 0; it < prm.iterations; it++)
    {
      const Point &p{pts[pool[pick(rng)]]}, &q{pts[pool[pick(rng)]]};
      const double dx{q.x - p.x}, dy{q.y - p.y}, len{std::hypot(dx, dy)};
      if(len < 2 * prm.tolerance) continue;
      const Line line{p, dx / len, dy / len};
      Run run{longestRun(pts, available, line, prm.tolerance, prm.max_gap)};
      if(isWall(run, prm) && run.members.size() > best.members.size())
      {
        best = std::move(run);
        best_line = line;
      }
    }
    if(best.members.empty()) break;

    // Polish with a least-squares fit; keep it only if it is still a wall.
    Line line{refit(pts, best.members)};
    Run run{longestRun(pts, available, line, prm.tolerance, prm.max_gap)};
    if(!isWall(run, prm))
    {
      line = best_line;
      run = std::move(best);
    }

    Wall wall;
    wall.a = {line.p.x + run.t0 * line.dx, line.p.y + run.t0 * line.dy};
    wall.b = {line.p.x + run.t1 * line.dx, line.p.y + run.t1 * line.dy};
    for(auto i : run.members) available[i] = 0;
    wall.members = std::move(run.members);
    walls.push_back(std::move(wall));
  }
  return walls;
}

// Labels for pts[subset[k]]: cluster id >= 0, or -1 for noise.
inline std::vector<int> dbscan(const std::vector<Point> &pts, const std::vector<size_t> &subset,
                               double eps, size_t min_pts)
{
  const size_t n{subset.size()};
  constexpr int kUnvisited{-2}, kNoise{-1};
  std::vector<int> label(n, kUnvisited);
  const double eps2{eps * eps};

  auto neighbours = [&](size_t k)
  {
    std::vector<size_t> out;
    for(size_t j = 0; j < n; j++)
    {
      const double dx{pts[subset[j]].x - pts[subset[k]].x};
      const double dy{pts[subset[j]].y - pts[subset[k]].y};
      if(dx * dx + dy * dy <= eps2) out.push_back(j);
    }
    return out;
  };

  int cluster{0};
  for(size_t k = 0; k < n; k++)
  {
    if(label[k] != kUnvisited) continue;
    auto seeds{neighbours(k)};
    if(seeds.size() < min_pts) { label[k] = kNoise; continue; }
    label[k] = cluster;
    for(size_t s = 0; s < seeds.size(); s++)
    {
      const size_t j{seeds[s]};
      if(label[j] == kNoise) label[j] = cluster;
      if(label[j] != kUnvisited) continue;
      label[j] = cluster;
      const auto more{neighbours(j)};
      if(more.size() >= min_pts) seeds.insert(seeds.end(), more.begin(), more.end());
    }
    cluster++;
  }
  return label;
}

}  // namespace sonar_geometry
