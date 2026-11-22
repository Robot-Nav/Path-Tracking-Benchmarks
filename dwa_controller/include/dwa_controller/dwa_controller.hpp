/**
 * @file dwa_controller.hpp
 * @brief Dynamic-window pure path tracking controller.
 */
#ifndef DWA_CONTROLLER_H_
#define DWA_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace dwa_controller
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2D
{
  double vx{0.0};
  double vz{0.0};
};

inline double normAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

namespace utils
{

inline Point2D getLookAheadPoint(
  double lookahead, const std::vector<Pose2D> & path, double & tangent_yaw)
{
  if (path.size() < 2) {
    tangent_yaw = path.empty() ? 0.0 : path.front().yaw;
    return path.empty() ? Point2D{} : Point2D{path.front().x, path.front().y};
  }
  double remaining = std::max(lookahead, 0.01);
  for (size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-9) continue;
    tangent_yaw = std::atan2(dy, dx);
    if (remaining <= length) {
      const double ratio = remaining / length;
      return {
        path[i - 1].x + ratio * dx,
        path[i - 1].y + ratio * dy};
    }
    remaining -= length;
  }
  tangent_yaw = path.back().yaw;
  return {path.back().x, path.back().y};
}

inline double projectToPath(
  double x, double y, const std::vector<Pose2D> & path, double & tangent_yaw)
{
  double best_distance_squared = std::numeric_limits<double>::max();
  tangent_yaw = path.empty() ? 0.0 : path.front().yaw;
  for (size_t i = 1; i < path.size(); ++i) {
    const double x0 = path[i - 1].x;
    const double y0 = path[i - 1].y;
    const double dx = path[i].x - x0;
    const double dy = path[i].y - y0;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1e-12) continue;
    const double ratio = std::clamp(
      ((x - x0) * dx + (y - y0) * dy) / length_squared, 0.0, 1.0);
    const double ex = x - (x0 + ratio * dx);
    const double ey = y - (y0 + ratio * dy);
    const double distance_squared = ex * ex + ey * ey;
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      tangent_yaw = std::atan2(dy, dx);
    }
  }
  if (best_distance_squared == std::numeric_limits<double>::max()) {
    return path.empty() ? 0.0 : std::hypot(x - path.front().x, y - path.front().y);
  }
  return std::sqrt(best_distance_squared);
}

}  // namespace utils

class DWAController
{
public:
  DWAController(
    double max_v, double min_v, double max_w, double lookahead,
    double acceleration_v, double acceleration_w, double control_dt,
    double prediction_time, int linear_samples, int angular_samples,
    double min_turning_radius = 0.5)
  : max_v_(std::max(0.0, max_v)),
    min_v_(std::clamp(min_v, 0.0, max_v_)),
    max_w_(std::max(0.0, max_w)),
    lookahead_(std::max(0.05, lookahead)),
    acceleration_v_(std::max(0.01, acceleration_v)),
    acceleration_w_(std::max(0.01, acceleration_w)),
    control_dt_(std::max(0.005, control_dt)),
    prediction_time_(std::max(0.1, prediction_time)),
    linear_samples_(std::max(2, linear_samples)),
    angular_samples_(std::max(3, angular_samples)),
    min_turning_radius_(std::max(1e-3, min_turning_radius))
  {
  }

  Twist2D compute(
    const Pose2D & current, const Twist2D & measured,
    const std::vector<Pose2D> & path) const
  {
    if (path.size() < 2) return {};

    double target_tangent = 0.0;
    const Point2D target = utils::getLookAheadPoint(lookahead_, path, target_tangent);
    const double bounded_v = std::clamp(measured.vx, min_v_, max_v_);
    const double bounded_w = std::clamp(measured.vz, -max_w_, max_w_);
    const double v_min = std::max(min_v_, bounded_v - acceleration_v_ * control_dt_);
    const double v_max = std::min(max_v_, bounded_v + acceleration_v_ * control_dt_);
    const double w_min = std::max(-max_w_, bounded_w - acceleration_w_ * control_dt_);
    const double w_max = std::min(max_w_, bounded_w + acceleration_w_ * control_dt_);

    double best_v = 0.0;
    double best_w = 0.0;
    double best_score = -std::numeric_limits<double>::max();
    for (int i = 0; i < linear_samples_; ++i) {
      const double v = v_min + (v_max - v_min) * i /
        static_cast<double>(linear_samples_ - 1);
      for (int j = 0; j < angular_samples_; ++j) {
        const double w = w_min + (w_max - w_min) * j /
          static_cast<double>(angular_samples_ - 1);
        if (std::abs(w) > v / min_turning_radius_ + 1e-9) continue;
        const double score = evaluate(v, w, current, target, target_tangent, path);
        if (score > best_score) {
          best_score = score;
          best_v = v;
          best_w = w;
        }
      }
    }
    return {best_v, best_w};
  }

private:
  double evaluate(
    double v, double w, const Pose2D & current, const Point2D & target,
    double target_tangent, const std::vector<Pose2D> & path) const
  {
    constexpr double simulation_dt = 0.05;
    const int steps = std::max(1, static_cast<int>(std::ceil(prediction_time_ / simulation_dt)));
    double x = current.x;
    double y = current.y;
    double yaw = current.yaw;
    double accumulated_path_error = 0.0;
    for (int step = 0; step < steps; ++step) {
      yaw = normAngle(yaw + w * simulation_dt);
      x += v * std::cos(yaw) * simulation_dt;
      y += v * std::sin(yaw) * simulation_dt;
      double path_yaw = 0.0;
      accumulated_path_error += utils::projectToPath(x, y, path, path_yaw);
    }

    double terminal_path_yaw = 0.0;
    const double terminal_path_error =
      utils::projectToPath(x, y, path, terminal_path_yaw);
    const double target_distance = std::hypot(target.x - x, target.y - y);
    const double heading_error = std::abs(normAngle(terminal_path_yaw - yaw));
    const double tangent_error = std::abs(normAngle(target_tangent - yaw));
    const double mean_path_error = accumulated_path_error / static_cast<double>(steps);
    const double speed_reward = max_v_ > 1e-6 ? v / max_v_ : 0.0;

    return 0.25 * speed_reward - 2.5 * mean_path_error -
      2.0 * terminal_path_error - 0.7 * target_distance -
      0.8 * heading_error - 0.25 * tangent_error;
  }

  double max_v_;
  double min_v_;
  double max_w_;
  double lookahead_;
  double acceleration_v_;
  double acceleration_w_;
  double control_dt_;
  double prediction_time_;
  int linear_samples_;
  int angular_samples_;
  double min_turning_radius_;
};

}  // namespace dwa_controller

#endif  // DWA_CONTROLLER_H_
