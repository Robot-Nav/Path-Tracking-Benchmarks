/**
 * @file apf_controller.hpp
 * @brief Attractive-field-only path tracking controller.
 */
#ifndef APF_CONTROLLER_H_
#define APF_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace apf_controller
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
    const double segment_length = std::hypot(dx, dy);
    if (segment_length <= 1e-9) continue;

    tangent_yaw = std::atan2(dy, dx);
    if (remaining <= segment_length) {
      const double ratio = remaining / segment_length;
      return {
        path[i - 1].x + ratio * dx,
        path[i - 1].y + ratio * dy};
    }
    remaining -= segment_length;
  }

  tangent_yaw = path.back().yaw;
  return {path.back().x, path.back().y};
}

}  // namespace utils

class APFController
{
public:
  APFController(
    double max_v, double min_v, double max_w, double lookahead,
    double heading_gain, int smooth_window)
  : max_v_(std::max(0.0, max_v)),
    min_v_(std::clamp(min_v, 0.0, max_v_)),
    max_w_(std::max(0.0, max_w)),
    lookahead_(std::max(0.05, lookahead)),
    heading_gain_(std::max(0.0, heading_gain)),
    smooth_window_(std::max(1, smooth_window))
  {
  }

  Twist2D compute(const Pose2D & current, const std::vector<Pose2D> & path)
  {
    if (path.size() < 2) return {};

    double path_tangent = 0.0;
    const Point2D target = utils::getLookAheadPoint(lookahead_, path, path_tangent);
    const double dx = target.x - current.x;
    const double dy = target.y - current.y;
    const double target_distance = std::hypot(dx, dy);
    if (target_distance <= 1e-6) return {};

    const double target_heading = std::atan2(dy, dx);
    const double pursuit_error = normAngle(target_heading - current.yaw);
    const double tangent_error = normAngle(path_tangent - current.yaw);

    // Attractive field provides pursuit; the tangent term suppresses corner cutting.
    double angular = heading_gain_ * pursuit_error + 0.35 * tangent_error;
    angular = std::clamp(angular, -max_w_, max_w_);

    angular_history_.push_back(angular);
    while (angular_history_.size() > static_cast<size_t>(smooth_window_)) {
      angular_history_.pop_front();
    }
    double smoothed_angular = 0.0;
    for (const double value : angular_history_) smoothed_angular += value;
    smoothed_angular /= static_cast<double>(angular_history_.size());

    const double heading_scale = std::clamp(std::cos(std::abs(pursuit_error)), 0.0, 1.0);
    const double distance_scale = std::clamp(target_distance / lookahead_, 0.0, 1.0);
    const double linear = std::clamp(
      max_v_ * heading_scale * distance_scale, min_v_, max_v_);

    return {linear, smoothed_angular};
  }

  void reset() { angular_history_.clear(); }

private:
  double max_v_;
  double min_v_;
  double max_w_;
  double lookahead_;
  double heading_gain_;
  int smooth_window_;
  std::deque<double> angular_history_;
};

}  // namespace apf_controller

#endif  // APF_CONTROLLER_H_
