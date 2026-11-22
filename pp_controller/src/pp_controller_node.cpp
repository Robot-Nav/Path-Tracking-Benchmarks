// SPDX-License-Identifier: GPL-3.0-only
/**
 * @file: pp_controller_node.cpp
 * @brief: ROS1 node for Graceful Pure Pursuit (PP) path tracking controller.
 *
 * Subscribes to /plan (nav_msgs/Path) and /odom1 (nav_msgs/Odometry),
 * publishes /cmd_vel (geometry_msgs/Twist), /local_path (nav_msgs/Path).
 *
 * Based on https://github.com/Robot-Nav/PP_Controller
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf2/utils.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "pp_controller/graceful_controller.hpp"

static inline nav_msgs::Path pose2dToPath(
    const std::vector<pp_controller::Pose2D>& pp,
    const std::string& frame) {
  nav_msgs::Path m; m.header.stamp=ros::Time::now(); m.header.frame_id=frame;
  for(auto& p:pp){geometry_msgs::PoseStamped ps; ps.header=m.header;
    ps.pose.position.x=p.position.x;ps.pose.position.y=p.position.y;
    ps.pose.orientation=tf::createQuaternionMsgFromYaw(p.orientation);m.poses.push_back(ps);}
  return m;
}

class PPControllerNode
{
public:
  PPControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
  {
    initParams();
    control_law_ = std::make_unique<pp_controller::SmoothControlLaw>(
        k_phi_, k_delta_, beta_, lambda_,
        slowdown_radius_, v_min_, v_max_, w_max_);

    path_sub_ = nh_.subscribe("/plan", 1, &PPControllerNode::pathCallback, this);
    odom_sub_ = nh_.subscribe("/odom1", 20, &PPControllerNode::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    solve_time_pub_ = nh_.advertise<std_msgs::Float64>("/controller_solve_time_ms", 100);
    target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_freq_),
        &PPControllerNode::controlLoop, this);

    ROS_INFO("PP Controller (Graceful Pure Pursuit) initialized.");
  }

private:
  void initParams()
  {
    pnh_.param("control_frequency", control_freq_, 20.0);

    // velocity limits
    pnh_.param("max_v", v_max_, 0.6);
    pnh_.param("min_v", v_min_, 0.05);
    pnh_.param("max_w", w_max_, 1.0);
    pnh_.param("min_turning_radius", min_turning_radius_, 0.5);

    // lookahead
    pnh_.param("lookahead_dist", lookahead_dist_, 0.6);

    // curvature control params (SmoothControlLaw)
    pnh_.param("k_phi", k_phi_, 2.0);
    pnh_.param("k_delta", k_delta_, 1.0);
    pnh_.param("beta", beta_, 0.4);
    pnh_.param("lambda", lambda_, 2.0);

    // slowdown
    pnh_.param("slowdown_radius", slowdown_radius_, 1.5);

    // goal tolerance
    pnh_.param("goal_dist_tolerance", goal_dist_tol_, 0.15);
    pnh_.param("goal_yaw_tolerance", goal_yaw_tol_, 0.2);

    // frame
    pnh_.param("planning_frame", planning_frame_, std::string("map"));
    pnh_.param("strict_frame_check", strict_frame_check_, true);

    // prune distance: max lookahead distance along path
    pnh_.param("prune_distance", prune_distance_, 5.0);
  }

  static double quaternionYaw(const geometry_msgs::Quaternion& q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.empty()) return;

    if (strict_frame_check_ && !msg->header.frame_id.empty() &&
        msg->header.frame_id != planning_frame_)
    {
      ROS_ERROR_THROTTLE(1.0, "PP: rejected path in frame '%s'; expected '%s'.",
                         msg->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }

    std::vector<pp_controller::Pose2D> path;
    path.reserve(msg->poses.size());
    for (const auto& ps : msg->poses)
    {
      path.emplace_back(ps.pose.position.x, ps.pose.position.y,
                        quaternionYaw(ps.pose.orientation));
    }

    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      global_path_ = std::move(path);
    }
    path_received_ = true;
    ROS_INFO("PP: Path accepted (%zu points).", global_path_.size());
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if (strict_frame_check_ && !msg->header.frame_id.empty() &&
        msg->header.frame_id != planning_frame_)
    {
      ROS_ERROR_THROTTLE(1.0, "PP: rejected odom in frame '%s'; expected '%s'.",
                         msg->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_ = pp_controller::Pose2D(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        quaternionYaw(msg->pose.pose.orientation));
    robot_speed_.vx = msg->twist.twist.linear.x;
    robot_speed_.vy = msg->twist.twist.linear.y;
    robot_speed_.wz = msg->twist.twist.angular.z;
    pose_received_ = true;
  }

  /**
   * @brief Prune path: remove points robot has passed, keeping up to prune_distance.
   */
  std::vector<pp_controller::Pose2D> prunePath(const pp_controller::Pose2D& robot)
  {
    std::vector<pp_controller::Pose2D> path;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      path = global_path_;
    }
    if (path.size() < 2) return path;

    // Find closest point on path
    size_t closest = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i)
    {
      double d = std::hypot(path[i].position.x - robot.position.x,
                            path[i].position.y - robot.position.y);
      if (d < min_dist)
      {
        min_dist = d;
        closest = i;
      }
    }

    // Collect points within prune_distance from closest point
    std::vector<pp_controller::Pose2D> pruned;
    double cum_dist = 0.0;
    for (size_t i = closest; i < path.size(); ++i)
    {
      pruned.push_back(path[i]);
      if (i > closest)
      {
        cum_dist += std::hypot(path[i].position.x - path[i - 1].position.x,
                               path[i].position.y - path[i - 1].position.y);
      }
      if (cum_dist > prune_distance_) break;
    }
    return pruned;
  }

  bool isGoalReached(const pp_controller::Pose2D& robot,
                     const pp_controller::Pose2D& goal)
  {
    double dist = std::hypot(robot.position.x - goal.position.x,
                             robot.position.y - goal.position.y);
    return dist < goal_dist_tol_;
  }

  void controlLoop(const ros::TimerEvent&)
  {
    if (!pose_received_ || !path_received_) return;

    pp_controller::Pose2D pose;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      pose = robot_pose_;
    }

    // Get pruned path and goal
    auto pruned = prunePath(pose);
    if (pruned.size() < 2)
    {
      publishStop();
      return;
    }

    pp_controller::Pose2D goal;
    { std::lock_guard<std::mutex> lock(path_mutex_); goal = global_path_.back(); }

    if (isGoalReached(pose, goal))
    {
      publishStop();
      return;
    }

    // Get lookahead point
    double theta_trj;
    pp_controller::Point2D lookahead_pt = pp_controller::utils::getLookAheadPoint(
        lookahead_dist_, pruned, pose.position.x, pose.position.y, theta_trj);

    // Target: lookahead position with tangent orientation
    pp_controller::Pose2D target(lookahead_pt.x, lookahead_pt.y, theta_trj);

    // Compute velocity command
    auto t1 = std::chrono::steady_clock::now();
    pp_controller::Twist2D cmd = control_law_->compute(target, pose);
    auto t2 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std_msgs::Float64 solve_message; solve_message.data = ms;
    solve_time_pub_.publish(solve_message);

    ROS_INFO("[PP] solve: %.3f ms | vx=%.3f wz=%.3f | lookahead=(%.2f,%.2f)",
             ms, cmd.vx, cmd.wz, lookahead_pt.x, lookahead_pt.y);

    geometry_msgs::Twist twist;
    twist.linear.x = std::clamp(cmd.vx, 0.0, v_max_);
    const double yaw_limit = std::min(w_max_, twist.linear.x / min_turning_radius_);
    twist.angular.z = std::clamp(cmd.wz, -yaw_limit, yaw_limit);
    cmd_vel_pub_.publish(twist);

    // Publish target point for visualization
    geometry_msgs::PointStamped pt_msg;
    pt_msg.header.stamp = ros::Time::now();
    pt_msg.header.frame_id = planning_frame_;
    pt_msg.point.x = lookahead_pt.x;
    pt_msg.point.y = lookahead_pt.y;
    target_pub_.publish(pt_msg);

    local_path_pub_.publish(pose2dToPath(pruned, planning_frame_));
  }

  void publishStop()
  {
    geometry_msgs::Twist stop;
    cmd_vel_pub_.publish(stop);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<pp_controller::SmoothControlLaw> control_law_;

  // Params
  double control_freq_;
  double v_max_, v_min_, w_max_, min_turning_radius_;
  double lookahead_dist_;
  double k_phi_, k_delta_, beta_, lambda_;
  double slowdown_radius_;
  double goal_dist_tol_, goal_yaw_tol_;
  double prune_distance_;
  std::string planning_frame_;
  bool strict_frame_check_;

  // State
  pp_controller::Pose2D robot_pose_;
  pp_controller::Twist2D robot_speed_;
  std::vector<pp_controller::Pose2D> global_path_;
  std::atomic<bool> pose_received_{false};
  std::atomic<bool> path_received_{false};

  std::mutex data_mutex_;
  std::mutex path_mutex_;

  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher solve_time_pub_;
  ros::Publisher target_pub_, local_path_pub_;
  ros::Timer control_timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pp_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  PPControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}
