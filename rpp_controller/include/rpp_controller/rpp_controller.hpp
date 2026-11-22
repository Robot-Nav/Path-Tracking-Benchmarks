/**
 * @file: rpp_controller.hpp
 * @brief: Regulated Pure Pursuit (RPP) controller.
 *         Adds curvature constraint, approach constraint, and
 *         optional scan-based obstacle constraint on top of pure pursuit.
 */
#ifndef RPP_CONTROLLER_H_
#define RPP_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rpp_controller
{

struct Point2D { double x, y; };
struct Pose2D { double x, y, yaw; };
struct Twist2D { double vx, vz; Twist2D() : vx(0), vz(0) {} };

inline double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

namespace utils
{

inline Point2D getLookAheadPoint(double L, const std::vector<Pose2D>& path,
                                  double rx, double ry, double& theta_out)
{
  if (path.size() < 2) { theta_out = 0.0; return {rx, ry}; }
  size_t idx = 0;
  for (; idx < path.size(); ++idx)
    if (std::hypot(path[idx].x - rx, path[idx].y - ry) >= L) break;

  if (idx >= path.size())
  {
    auto& p = path.back();
    theta_out = std::atan2(p.y - ry, p.x - rx);
    return {p.x, p.y};
  }

  double px, py;
  if (idx == 0) { px = rx; py = ry; }
  else { px = path[idx - 1].x; py = path[idx - 1].y; }
  double gx = path[idx].x, gy = path[idx].y;

  // Circle-segment intersection
  double rpx = px - rx, rpy = py - ry, rgx = gx - rx, rgy = gy - ry;
  double dx = rgx - rpx, dy = rgy - rpy;
  double d2 = dx * dx + dy * dy;
  double det = rpx * rgy - rgx * rpy;
  double disc = L * L * d2 - det * det;

  if (disc < 0) { theta_out = std::atan2(gy - py, gx - px); return {gx, gy}; }

  double sq = std::sqrt(disc);
  double sy = dy < 0 ? -1.0 : 1.0;
  double ix1 = (det * dy + sy * dx * sq) / d2 + rx;
  double iy1 = (-det * dx + std::abs(dy) * sq) / d2 + ry;
  double ix2 = (det * dy - sy * dx * sq) / d2 + rx;
  double iy2 = (-det * dx - std::abs(dy) * sq) / d2 + ry;

  theta_out = std::atan2(gy - py, gx - px);
  return (std::hypot(ix1 - gx, iy1 - gy) < std::hypot(ix2 - gx, iy2 - gy))
      ? Point2D{ix1, iy1} : Point2D{ix2, iy2};
}

}  // namespace utils

class RPPController
{
public:
  RPPController(double max_v, double min_v, double max_w,
                double lookahead_time, double min_lookahead, double max_lookahead,
                double regulated_min_radius, double inflation_cost_factor,
                double scaling_dist, double scaling_gain,
                double approach_dist, double approach_min_v)
    : max_v_(max_v), min_v_(min_v), max_w_(max_w),
      lookahead_time_(lookahead_time), min_lookahead_(min_lookahead),
      max_lookahead_(max_lookahead),
      regulated_min_radius_(regulated_min_radius),
      inflation_cost_factor_(inflation_cost_factor),
      scaling_dist_(scaling_dist), scaling_gain_(scaling_gain),
      approach_dist_(approach_dist), approach_min_v_(approach_min_v) {}

  Twist2D compute(const Pose2D& current, const std::vector<Pose2D>& path,
                  double vt, double obstacle_cost = 0.0,
                  double inscribed_radius = 0.2)
  {
    if (path.size() < 2) return Twist2D();

    // Dynamic lookahead distance
    double lookahead = std::abs(vt) * lookahead_time_;
    lookahead = std::clamp(lookahead, min_lookahead_, max_lookahead_);

    double theta_trj;
    Point2D lp = utils::getLookAheadPoint(lookahead, path, current.x, current.y, theta_trj);

    // Curvature from lookahead angle
    double dphi = std::atan2(lp.y - current.y, lp.x - current.x) - current.yaw;
    dphi = normalizeAngle(dphi);
    double lookahead_k = 2.0 * std::sin(dphi) / lookahead;

    // Apply constraints
    double curv_vel = applyCurvatureConstraint(max_v_, lookahead_k);
    double cost_vel = applyObstacleConstraint(max_v_, obstacle_cost, inscribed_radius);
    double vd = std::min(curv_vel, cost_vel);
    vd = applyApproachConstraint(vd, current, path);

    vd = std::clamp(vd, min_v_, max_v_);
    double wd = vd * lookahead_k;
    wd = std::clamp(wd, -max_w_, max_w_);

    Twist2D cmd;
    cmd.vx = vd;
    cmd.vz = wd;
    return cmd;
  }

private:
  double applyCurvatureConstraint(double raw_v, double curvature)
  {
    double radius = std::abs(1.0 / (std::abs(curvature) > 1e-9 ? curvature : 1e-9));
    if (radius < regulated_min_radius_)
      return raw_v * radius / regulated_min_radius_;
    return raw_v;
  }

  double applyObstacleConstraint(double raw_v, double robot_cost, double inscribed_radius)
  {
    // Heuristic based on costmap cost (if available)
    const double LETHAL = 254.0;
    const double INSCRIBED = 253.0;
    const double FREE = 0.0;
    if (robot_cost <= FREE + 1e-6 || robot_cost >= LETHAL - 1e-6)
      return raw_v;

    double obs_dist = inscribed_radius -
        (std::log(robot_cost) - std::log(INSCRIBED)) / inflation_cost_factor_;

    if (obs_dist < scaling_dist_)
      return raw_v * scaling_gain_ * obs_dist / scaling_dist_;
    return raw_v;
  }

  double applyApproachConstraint(double raw_v, const Pose2D& robot,
                                  const std::vector<Pose2D>& path)
  {
    double remain = 0.0;
    auto dist = [](const Pose2D& a, const Pose2D& b) {
      return std::hypot(a.x - b.x, a.y - b.y);
    };
    for (size_t i = 0; i + 1 < path.size(); ++i)
      remain += dist(path[i], path[i + 1]);

    if (remain < approach_dist_)
    {
      double s = dist(path.back(), robot) / approach_dist_;
      return std::min(raw_v, std::max(approach_min_v_, raw_v * s));
    }
    return raw_v;
  }

  double max_v_, min_v_, max_w_;
  double lookahead_time_, min_lookahead_, max_lookahead_;
  double regulated_min_radius_;
  double inflation_cost_factor_;
  double scaling_dist_, scaling_gain_;
  double approach_dist_, approach_min_v_;
};

}  // namespace rpp_controller
#endif  // RPP_CONTROLLER_H_
