// survey_plan.hpp
//
// Generates the waypoints of the submerged survey. Kept free of ROS so the
// survey can be reworked and checked without touching task_node's sequencing.

#ifndef JIT_TASK__SURVEY_PLAN_HPP_
#define JIT_TASK__SURVEY_PLAN_HPP_

#include <cmath>
#include <vector>

namespace jit_task
{

struct SurveyLeg
{
  double x;      // metres East in local_origin, absolute
  double y;      // metres North in local_origin, absolute
  bool has_z;    // false: hold the depth of the previous leg
  double z;      // metres Up from the depth at mission start (negative = deeper)
};

// Dive in place at (start_x, start_y), traverse length_m along heading_rad at
// that depth, then rise in place. heading_rad is ENU: 0 = East, counter-
// clockwise. Each leg either changes depth or translates, never both.
inline std::vector<SurveyLeg> make_survey(
  double start_x, double start_y, double heading_rad, double depth_m, double length_m)
{
  const double end_x = start_x + length_m * std::cos(heading_rad);
  const double end_y = start_y + length_m * std::sin(heading_rad);
  return {
    {start_x, start_y, true, -depth_m},
    {end_x, end_y, false, 0.0},
    {end_x, end_y, true, 0.0},
  };
}

}  // namespace jit_task

#endif  // JIT_TASK__SURVEY_PLAN_HPP_
