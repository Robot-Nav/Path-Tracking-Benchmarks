// SPDX-License-Identifier: GPL-3.0-only
/**
 * @file: graceful_controller.hpp
 * @brief: Graceful Pure Pursuit controller - curvature-based path tracking
 *         Adapted from https://github.com/Robot-Nav/PP_Controller
 *
 * Core algorithm: SmoothControlLaw using egocentric polar coordinates
 * to compute curvature, with velocity scaling based on curvature and
 * slowdown near goal.
 */
#ifndef PP_CONTROLLER_GRACEFUL_CONTROLLER_H_
#define PP_CONTROLLER_GRACEFUL_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pp_controller
{

// ============ Basic Types ============

struct Point2D
{
  double x, y;
  Point2D() : x(0.0), y(0.0) {}
  Point2D(double x_, double y_) : x(x_), y(y_) {}
  double length() const { return std::hypot(x, y); }
};

struct Pose2D
{
  Point2D position;
  double orientation;  // yaw angle [rad]
  Pose2D() : position(), orientation(0.0) {}
  Pose2D(double x, double y, double yaw) : position(x, y), orientation(yaw) {}
};

struct Twist2D
{
  double vx, vy, wz;
  Twist2D() : vx(0.0), vy(0.0), wz(0.0) {}
};

// ============ Utility Functions ============

inline double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

namespace utils
{

/**
 * @brief Circle-segment intersection: find the point on segment p1->p2
 *        that lies at distance r from the origin.
 */
inline Point2D circleSegmentIntersection(const Point2D& p1, const Point2D& p2, double r)
{
  double dx = p2.x - p1.x, dy = p2.y - p1.y;
  double d2 = dx * dx + dy * dy;
  double det = p1.x * p2.y - p2.x * p1.y;
  double disc = r * r * d2 - det * det;
  if (disc < 0) return p2;

  double sqrt_disc = std::sqrt(disc);
  double sign_dy = dy < 0 ? -1.0 : 1.0;
  double x1 = (det * dy + sign_dy * dx * sqrt_disc) / d2;
  double y1 = (-det * dx + std::abs(dy) * sqrt_disc) / d2;
  double x2 = (det * dy - sign_dy * dx * sqrt_disc) / d2;
  double y2 = (-det * dx - std::abs(dy) * sqrt_disc) / d2;

  double d1 = std::hypot(x1 - p2.x, y1 - p2.y);
  double d2_val = std::hypot(x2 - p2.x, y2 - p2.y);
  return (d1 < d2_val) ? Point2D(x1, y1) : Point2D(x2, y2);
}

/**
 * @brief Get lookahead point on a path at given distance from origin.
 * @param lookahead_dist  desired lookahead distance
 * @param path            the path (poses in global frame)
 * @param robot_x         robot x position
 * @param robot_y         robot y position
 * @param theta_out       output: path tangent angle at lookahead point
 * @return the world-frame lookahead point
 */
inline Point2D getLookAheadPoint(double lookahead_dist,
                                  const std::vector<Pose2D>& path,
                                  double robot_x, double robot_y,
                                  double& theta_out)
{
  if (path.empty())
  {
    theta_out = 0.0;
    return Point2D(robot_x, robot_y);
  }

  // Find first pose farther than lookahead distance from robot
  size_t idx = 0;
  for (; idx < path.size(); ++idx)
  {
    if (std::hypot(path[idx].position.x - robot_x,
                   path[idx].position.y - robot_y) >= lookahead_dist)
      break;
  }

  if (idx >= path.size())
  {
    // All points inside circle: take last point
    const auto& last = path.back();
    theta_out = std::atan2(last.position.y - robot_y, last.position.x - robot_x);
    return Point2D(last.position.x, last.position.y);
  }

  const auto& goal_pose = path[idx];
  Point2D prev_p, goal_p;
  if (idx == 0)
  {
    prev_p = Point2D(robot_x, robot_y);
  }
  else
  {
    prev_p = Point2D(path[idx - 1].position.x, path[idx - 1].position.y);
  }
  goal_p = Point2D(goal_pose.position.x, goal_pose.position.y);

  // Transform to robot frame
  Point2D p1(prev_p.x - robot_x, prev_p.y - robot_y);
  Point2D p2(goal_p.x - robot_x, goal_p.y - robot_y);

  Point2D intersection = circleSegmentIntersection(p1, p2, lookahead_dist);

  // Tangent angle
  theta_out = std::atan2(goal_p.y - prev_p.y, goal_p.x - prev_p.x);

  return Point2D(intersection.x + robot_x, intersection.y + robot_y);
}

/**
 * @brief Calculate path length from a given index.
 */
inline double calculatePathLength(const std::vector<Pose2D>& path, size_t start_idx = 0)
{
  double length = 0.0;
  for (size_t i = start_idx + 1; i < path.size(); ++i)
    length += std::hypot(path[i].position.x - path[i - 1].position.x,
                          path[i].position.y - path[i - 1].position.y);
  return length;
}

}  // namespace utils

// ============ SmoothControlLaw ============

/**
 * @brief The core graceful pure pursuit control law.
 *
 * Computes curvature from egocentric polar coordinates (r, phi, delta):
 *   curvature = -(k_delta * (delta - atan(-k_phi * phi)) +
 *                 (1 + k_phi / (1 + (k_phi*phi)^2)) * sin(delta)) / r
 *
 * Velocity is scaled by curvature and slowdown near goal.
 */
class SmoothControlLaw
{
public:
  SmoothControlLaw(double k_phi, double k_delta, double beta, double lambda,
                   double slowdown_radius, double v_min, double v_max, double w_max)
    : k_phi_(k_phi), k_delta_(k_delta), beta_(beta), lambda_(lambda),
      slowdown_radius_(slowdown_radius), v_min_(v_min), v_max_(v_max), w_max_(w_max) {}

  /**
   * @brief Compute velocity command to track a target pose.
   * @param target   target pose (world frame)
   * @param current  current robot pose (world frame)
   * @return velocity command [vx, wz]
   */
  Twist2D compute(const Pose2D& target, const Pose2D& current)
  {
    // Transform target to egocentric polar coordinates
    double dx = target.position.x - current.position.x;
    double dy = target.position.y - current.position.y;
    double r = std::hypot(dx, dy);

    if (r < std::numeric_limits<double>::epsilon())
      return Twist2D();

    double line_of_sight = std::atan2(dy, dx);
    double phi = normalizeAngle(target.orientation - line_of_sight);
    double delta = normalizeAngle(current.orientation - line_of_sight);

    double curvature = calculateCurvature(r, phi, delta);

    // Velocity from curvature
    double v = v_max_;
    if (std::abs(curvature) > std::numeric_limits<double>::epsilon())
      v = v_max_ / (1.0 + beta_ * std::pow(std::abs(curvature), lambda_));

    // Slowdown near goal
    if (slowdown_radius_ > 0.0 && r < slowdown_radius_)
    {
      double factor = r / slowdown_radius_;
      v = std::min(v_max_ * factor, v);
    }

    v = std::clamp(v, v_min_, v_max_);
    double w = curvature * v;
    w = std::clamp(w, -w_max_, w_max_);

    Twist2D cmd;
    cmd.vx = v;
    cmd.wz = w;
    return cmd;
  }

  void setParams(double k_phi, double k_delta, double beta, double lambda)
  {
    k_phi_ = k_phi; k_delta_ = k_delta; beta_ = beta; lambda_ = lambda;
  }

  void setSpeedLimits(double v_min, double v_max, double w_max)
  {
    v_min_ = v_min; v_max_ = v_max; w_max_ = w_max;
  }

  void setSlowdownRadius(double r) { slowdown_radius_ = r; }

private:
  double calculateCurvature(double r, double phi, double delta)
  {
    if (r < 1e-9) return 0.0;

    double prop = k_delta_ * (delta - std::atan(-k_phi_ * phi));
    double denom = 1.0 + std::pow(k_phi_ * phi, 2);
    double feedback = (1.0 + k_phi_ / denom) * std::sin(delta);
    double curvature = -(prop + feedback) / r;

    if (std::isnan(curvature) || std::isinf(curvature))
      return 0.0;
    return curvature;
  }

  double k_phi_, k_delta_, beta_, lambda_;
  double slowdown_radius_;
  double v_min_, v_max_, w_max_;
};

}  // namespace pp_controller

#endif  // PP_CONTROLLER_GRACEFUL_CONTROLLER_H_
