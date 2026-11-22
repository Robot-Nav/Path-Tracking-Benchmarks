/**
 * @file: pid_controller.hpp
 * @brief: PID path tracking controller using inverse kinematics.
 *         Converts position error (in map frame) to velocity commands.
 *         Adapted from local PID controller in rmp::controller::PIDController.
 */
#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pid_controller
{

struct Point2D { double x, y; };
struct Pose2D { Point2D position; double yaw; };
struct Twist2D { double vx, vz; Twist2D() : vx(0), vz(0) {} };

inline double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

namespace utils
{

inline Point2D getLookAheadPoint(double lookahead_dist,
                                  const std::vector<Pose2D>& path,
                                  double rx, double ry,
                                  double& theta_out)
{
  if (path.empty()) { theta_out = 0.0; return {rx, ry}; }

  size_t idx = 0;
  for (; idx < path.size(); ++idx)
    if (std::hypot(path[idx].position.x - rx, path[idx].position.y - ry) >= lookahead_dist)
      break;

  if (idx >= path.size())
  {
    auto& last = path.back();
    theta_out = std::atan2(last.position.y - ry, last.position.x - rx);
    return {last.position.x, last.position.y};
  }

  // Translate to robot frame for circle-segment intersection
  auto& gp = path[idx];
  auto& pp = (idx == 0) ? path[idx] : path[idx - 1];
  double px = pp.position.x, py = pp.position.y;
  if (idx == 0) { px = rx; py = ry; }

  double gx = gp.position.x, gy = gp.position.y;
  double dx = gx - px, dy = gy - py;
  double d2 = dx * dx + dy * dy;

  double det = (px - rx) * (gy - ry) - (gx - rx) * (py - ry);
  double disc = lookahead_dist * lookahead_dist * d2 - det * det;
  if (disc < 0) return {gx, gy};

  double sqrt_disc = std::sqrt(disc);
  double sy = dy < 0 ? -1.0 : 1.0;

  double ix1 = (det * dy + sy * dx * sqrt_disc) / d2 + rx;
  double iy1 = (-det * dx + std::abs(dy) * sqrt_disc) / d2 + ry;
  double ix2 = (det * dy - sy * dx * sqrt_disc) / d2 + rx;
  double iy2 = (-det * dx - std::abs(dy) * sqrt_disc) / d2 + ry;

  double d1 = std::hypot(ix1 - gx, iy1 - gy);
  double d2v = std::hypot(ix2 - gx, iy2 - gy);

  theta_out = std::atan2(gy - py, gx - px);
  return (d1 < d2v) ? Point2D{ix1, iy1} : Point2D{ix2, iy2};
}

} // namespace utils

/**
 * @brief PID controller using kinematic model inverse.
 *
 * The control law transforms position error into velocity:
 *   [v, w]^T = R^{-1} * [k*e_x, k*e_y]^T
 *   where R^{-1} = [ cos(theta), sin(theta);
 *                   -sin(theta)/L,  cos(theta)/L ]
 *
 * L is the wheelbase parameter (lookahead distance surrogate).
 */
class PIDController
{
public:
  PIDController(double max_v, double min_v, double max_w,
                double k, double L, double lookahead_dist)
    : max_v_(max_v), min_v_(min_v), max_w_(max_w),
      k_(k), L_(L), lookahead_dist_(lookahead_dist) {}

  Twist2D compute(const Pose2D& current, const std::vector<Pose2D>& path)
  {
    if (path.size() < 2) return Twist2D();

    double theta_trj;
    Point2D lookahead = utils::getLookAheadPoint(
        lookahead_dist_, path, current.position.x, current.position.y, theta_trj);

    double ex = lookahead.x - current.position.x;
    double ey = lookahead.y - current.position.y;

    double sx_dot_x = k_ * ex;
    double sx_dot_y = k_ * ey;

    double ct = std::cos(current.yaw);
    double st = std::sin(current.yaw);

    // Inverse kinematics
    double v = ct * sx_dot_x + st * sx_dot_y;
    double w = (-st * sx_dot_x + ct * sx_dot_y) / L_;

    v = clamp(v, min_v_, max_v_);
    w = clamp(w, -max_w_, max_w_);

    Twist2D cmd;
    cmd.vx = v;
    cmd.vz = w;
    return cmd;
  }

private:
  double max_v_, min_v_, max_w_;
  double k_, L_;
  double lookahead_dist_;
};

}  // namespace pid_controller

#endif  // PID_CONTROLLER_H_
