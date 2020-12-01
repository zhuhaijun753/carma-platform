#include <safe_stop/smoothing/CubicSpline.h>

namespace safe_stop_planner
{
namespace smoothing
{
void CubicSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  std::vector<double> x;
  std::vector<double> y;
  for (const auto& p : points)
  {
    x.push_back(p.x());
    y.push_back(p.y());
  }
  spline_.set_points(x, y, true);
}
double CubicSpline::operator()(double x) const
{
  return spline_(x);
}
};  // namespace smoothing
};  // namespace safe_stop_planner