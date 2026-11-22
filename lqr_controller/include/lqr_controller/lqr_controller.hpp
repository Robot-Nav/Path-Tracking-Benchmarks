/**
 * @file: lqr_controller.hpp
 * @brief: Linear Quadratic Regulator (LQR) path tracking controller.
 *         Uses discrete-time Riccati equation for optimal feedback gain.
 */
#ifndef LQR_CONTROLLER_H_
#define LQR_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lqr_controller
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
                                  double rx, double ry, double& theta_trj, double& kappa)
{
  if (path.size() < 2) { theta_trj = 0.0; kappa = 0.0; return {rx, ry}; }

  size_t idx = 0;
  for (; idx < path.size(); ++idx)
    if (std::hypot(path[idx].x - rx, path[idx].y - ry) >= L) break;

  if (idx >= path.size())
  {
    auto& p = path.back();
    theta_trj = std::atan2(p.y - ry, p.x - rx);
    kappa = 0.0;
    return {p.x, p.y};
  }

  double px, py;
  if (idx == 0) { px = rx; py = ry; }
  else { px = path[idx - 1].x; py = path[idx - 1].y; }
  double gx = path[idx].x, gy = path[idx].y;

  // Circle-segment intersection in robot frame
  double rpx = px - rx, rpy = py - ry;
  double rgx = gx - rx, rgy = gy - ry;
  double dx = rgx - rpx, dy = rgy - rpy;
  double d2 = dx * dx + dy * dy;
  double det = rpx * rgy - rgx * rpy;
  double disc = L * L * d2 - det * det;

  double ix, iy;
  if (disc < 0)
  {
    ix = gx; iy = gy;
  }
  else
  {
    double sq = std::sqrt(disc);
    double sy = dy < 0 ? -1.0 : 1.0;
    double ix1 = (det * dy + sy * dx * sq) / d2 + rx;
    double iy1 = (-det * dx + std::abs(dy) * sq) / d2 + ry;
    double ix2 = (det * dy - sy * dx * sq) / d2 + rx;
    double iy2 = (-det * dx - std::abs(dy) * sq) / d2 + ry;
    if (std::hypot(ix1 - gx, iy1 - gy) < std::hypot(ix2 - gx, iy2 - gy))
      { ix = ix1; iy = iy1; }
    else
      { ix = ix2; iy = iy2; }
  }

  theta_trj = std::atan2(gy - py, gx - px);

  // Curvature from 3-point arc
  if (idx + 1 < path.size())
  {
    double x1 = px, y1 = py, x2 = gx, y2 = gy;
    double x3 = path[idx + 1].x, y3 = path[idx + 1].y;
    double a = std::hypot(x2 - x1, y2 - y1);
    double b = std::hypot(x3 - x2, y3 - y2);
    double c = std::hypot(x3 - x1, y3 - y1);
    double s = (a + b + c) / 2.0;
    double area = std::sqrt(s * (s - a) * (s - b) * (s - c));
    if (area > 1e-9)
    {
      double R = a * b * c / (4.0 * area);
      double cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
      kappa = (cross > 0) ? 1.0 / R : -1.0 / R;
    }
    else kappa = 0.0;
  }
  else kappa = 0.0;

  return {ix, iy};
}

} // namespace utils

class LQRController
{
public:
  LQRController(const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R,
                double dt, double max_v, double min_v, double max_w,
                double lookahead_dist, int max_iter = 100, double eps = 1e-4)
    : Q_(Q), R_(R), dt_(dt), max_v_(max_v), min_v_(min_v), max_w_(max_w),
      lookahead_dist_(lookahead_dist), max_iter_(max_iter), eps_(eps)
  {
    computeK();
  }

  Twist2D compute(const Pose2D& current, const std::vector<Pose2D>& path,
                  double vt = 0.0)
  {
    if (path.size() < 2) return Twist2D();

    double theta_trj, kappa;
    Point2D lookahead = utils::getLookAheadPoint(
        lookahead_dist_, path, current.x, current.y, theta_trj, kappa);

    // Desired state
    Eigen::Vector3d s_d(lookahead.x, lookahead.y, theta_trj);
    // Current state
    Eigen::Vector3d s(current.x, current.y, current.yaw);
    // Reference control
    Eigen::Vector2d u_r(vt, vt * kappa);

    Eigen::Vector3d e = s - s_d;
    e(2) = normalizeAngle(e(2));

    Eigen::Vector2d u = u_r + K_ * e;

    u(0) = std::clamp(u(0), min_v_, max_v_);
    u(1) = std::clamp(u(1), -max_w_, max_w_);

    Twist2D cmd;
    cmd.vx = u(0);
    cmd.vz = u(1);
    return cmd;
  }

  void updateQR(const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R)
  {
    Q_ = Q; R_ = R;
    computeK();
  }

private:
  void computeK()
  {
    // Discrete LQR via Riccati iteration
    // System: e_{k+1} = A_k * e_k + B_k * u_k
    // A_k = I + dt * [0 0 -u_r*v; 0 0 u_r*v; 0 0 0]  -- linearized at ref trajectory
    // B_k = dt * [cos(theta_d) 0; sin(theta_d) 0; 0 1]
    // For simplified computation, use the linearized form at identity:
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
    B(0, 0) = dt_;
    B(1, 0) = 0.0;
    B(2, 1) = dt_;

    // Treat A as identity for simplified case (small dt, straight line)
    // More generally: A(0,2) = -vt * sin(theta_d) * dt, A(1,2) = vt * cos(theta_d) * dt

    Eigen::Matrix3d P = Q_;
    for (int i = 0; i < max_iter_; ++i)
    {
      Eigen::Matrix2d temp = R_ + B.transpose() * P * B;
      Eigen::Matrix3d P_next = Q_ + A.transpose() * P * A -
          A.transpose() * P * B * temp.inverse() * B.transpose() * P * A;
      if ((P - P_next).array().abs().maxCoeff() < eps_)
      {
        P = P_next;
        break;
      }
      P = P_next;
    }

    K_ = -(R_ + B.transpose() * P * B).inverse() * B.transpose() * P * A;
  }

  Eigen::Matrix3d Q_;
  Eigen::Matrix2d R_;
  Eigen::MatrixXd K_;

  double dt_, max_v_, min_v_, max_w_;
  double lookahead_dist_;
  int max_iter_;
  double eps_;
};

}  // namespace lqr_controller

#endif  // LQR_CONTROLLER_H_
