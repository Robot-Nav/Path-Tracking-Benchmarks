/**
 * @file: mpc_controller.hpp
 * @brief: Model Predictive Control (MPC) path tracking controller.
 *         Properly tracks path by generating reference points along the path
 *         for each prediction step. Uses grid search over velocity space.
 */
#ifndef MPC_CONTROLLER_H_
#define MPC_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mpc_controller
{

struct Point2D { double x, y; };
struct Pose2D { double x, y, yaw; };
struct Twist2D { double vx, vz; Twist2D() : vx(0), vz(0) {} };

inline double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

namespace utils {

inline Point2D getLookAheadPoint(double L, const std::vector<Pose2D>& path,
    double rx, double ry, double& theta) {
  if (path.size() < 2) { theta=0; return {rx,ry}; }
  size_t idx=0;
  for (; idx<path.size(); ++idx)
    if (std::hypot(path[idx].x-rx, path[idx].y-ry) >= L) break;
  if (idx>=path.size()) { auto& p=path.back(); theta=p.yaw; return {p.x,p.y}; }
  double px=(idx==0)?rx:path[idx-1].x, py=(idx==0)?ry:path[idx-1].y;
  double gx=path[idx].x, gy=path[idx].y;
  double rpx=px-rx, rpy=py-ry, rgx=gx-rx, rgy=gy-ry;
  double dx=rgx-rpx, dy=rgy-rpy, d2=dx*dx+dy*dy;
  double det=rpx*rgy-rgx*rpy, disc=L*L*d2-det*det;
  theta=path[idx].yaw;  // use path orientation at this segment
  if (disc<0) return {gx,gy};
  double sq=std::sqrt(disc), sy=dy<0?-1.0:1.0;
  double ix1=(det*dy+sy*dx*sq)/d2+rx, iy1=(-det*dx+std::abs(dy)*sq)/d2+ry;
  double ix2=(det*dy-sy*dx*sq)/d2+rx, iy2=(-det*dx-std::abs(dy)*sq)/d2+ry;
  return (std::hypot(ix1-gx,iy1-gy) < std::hypot(ix2-gx,iy2-gy)) ? Point2D{ix1,iy1} : Point2D{ix2,iy2};
}

/// Collect reference points along path at regular intervals from current lookahead
inline std::vector<Pose2D> generateReferenceTrajectory(
    const std::vector<Pose2D>& path, double lookahead,
    int N, double spacing) {
  std::vector<Pose2D> refs;
  if (path.size() < 2) return refs;
  refs.reserve(N);
  for (int k = 0; k < N; ++k) {
    double d = lookahead + k * spacing;
    double theta;
    Point2D pt = getLookAheadPoint(d, path, path.front().x, path.front().y, theta);
    refs.push_back({pt.x, pt.y, theta});
  }
  return refs;
}

}  // namespace utils

class MPCController {
public:
  MPCController(double dt, int horizon,
                double max_v, double min_v, double max_w,
                double lookahead, double acceleration_v = 1.2,
                double acceleration_w = 2.5,
                double min_turning_radius = 0.5)
    : dt_(dt), N_(horizon),
      max_v_(max_v), min_v_(min_v), max_w_(max_w),
      lookahead_(lookahead), acceleration_v_(acceleration_v),
      acceleration_w_(acceleration_w), min_turning_radius_(min_turning_radius),
      // Cost weights
      w_pos_(1.0), w_theta_(0.5), w_vel_(0.1), w_omega_(0.05),
      prev_v_(0), prev_w_(0) {}

  Twist2D compute(const Pose2D& current, const std::vector<Pose2D>& path,
                  double /* vt unused */) {
    if (path.size() < 2) return Twist2D();

    // Generate reference trajectory: N reference points along the path
    double ref_spacing = max_v_ * dt_;  // space between reference points
    auto refs = utils::generateReferenceTrajectory(path, lookahead_, N_, ref_spacing);
    if ((int)refs.size() < N_) return Twist2D();

    // ========== Grid search over velocity space ==========
    // Dynamic window
    double v_min = std::max(min_v_, prev_v_ - acceleration_v_ * dt_);
    double v_max = std::min(max_v_, prev_v_ + acceleration_v_ * dt_);
    double w_min = std::max(-max_w_, prev_w_ - acceleration_w_ * dt_);
    double w_max = std::min( max_w_, prev_w_ + acceleration_w_ * dt_);

    int nv = 20, nw = 25;
    double best_v = 0.0, best_w = 0.0;
    double best_cost = std::numeric_limits<double>::max();

    for (int i = 0; i < nv; ++i) {
      double v = v_min + (v_max - v_min) * i / (nv - 1);
      for (int j = 0; j < nw; ++j) {
        double w = w_min + (w_max - w_min) * j / (nw - 1);
        if (std::fabs(w) > v / min_turning_radius_ + 1e-9) continue;
        double cost = rolloutCost(current, refs, v, w);
        if (cost < best_cost) {
          best_cost = cost;
          best_v = v;
          best_w = w;
        }
      }
    }

    prev_v_ = best_v;
    prev_w_ = best_w;
    Twist2D cmd;
    cmd.vx = best_v;
    cmd.vz = best_w;
    return cmd;
  }

private:
  /// Simulate N steps forward with (v,w), compare each step against its reference
  double rolloutCost(const Pose2D& s0, const std::vector<Pose2D>& refs,
                     double v, double w) {
    double cost = 0.0;
    double x = s0.x, y = s0.y, th = s0.yaw;
    for (int k = 0; k < N_; ++k) {
      // Simulate one step
      x += v * std::cos(th) * dt_;
      y += v * std::sin(th) * dt_;
      th += w * dt_;
      th = normalizeAngle(th);

      // Error relative to k-th reference
      const auto& r = refs[k];
      double ex = x - r.x;
      double ey = y - r.y;
      double eth = normalizeAngle(th - r.yaw);
      double pos_err = std::hypot(ex, ey);

      // Accumulate cost with exponential weight (later steps matter more)
      double gamma = 1.0 + 0.3 * k;  // increasing weight
      cost += gamma * (w_pos_ * pos_err * pos_err +
                        w_theta_ * eth * eth);
    }
    // Control effort penalty
    cost += w_vel_ * v * v + w_omega_ * w * w;
    return cost;
  }

  double dt_;
  int N_;
  double max_v_, min_v_, max_w_, lookahead_;
  double acceleration_v_, acceleration_w_, min_turning_radius_;
  double w_pos_, w_theta_, w_vel_, w_omega_;
  double prev_v_, prev_w_;
};

}  // namespace mpc_controller
#endif
