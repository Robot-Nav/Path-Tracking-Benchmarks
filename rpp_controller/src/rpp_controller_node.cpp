/**
 * @file: rpp_controller_node.cpp
 * @brief: ROS1 node for Regulated Pure Pursuit (RPP) path tracking controller.
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/tf.h>
#include <tf2/utils.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rpp_controller/rpp_controller.hpp"

static inline nav_msgs::Path pose2dToPath(
    const std::vector<rpp_controller::Pose2D>& pp,
    const std::string& frame) {
  nav_msgs::Path m; m.header.stamp=ros::Time::now(); m.header.frame_id=frame;
  for(auto& p:pp){geometry_msgs::PoseStamped ps; ps.header=m.header;
    ps.pose.position.x=p.x;ps.pose.position.y=p.y;
    ps.pose.orientation=tf::createQuaternionMsgFromYaw(p.yaw);m.poses.push_back(ps);}
  return m;
}

class RPPControllerNode
{
public:
  RPPControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
  {
    initParams();
    controller_ = std::make_unique<rpp_controller::RPPController>(
        max_v_, min_v_, max_w_, lookahead_time_, min_lookahead_, max_lookahead_,
        min_radius_, infl_factor_, scaling_dist_, scaling_gain_,
        approach_dist_, approach_min_v_);

    path_sub_  = nh_.subscribe("/plan", 1, &RPPControllerNode::pathCallback, this);
    odom_sub_  = nh_.subscribe("/odom1", 20, &RPPControllerNode::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    solve_time_pub_ = nh_.advertise<std_msgs::Float64>("/controller_solve_time_ms", 100);
    target_pub_  = nh_.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);

    if (use_scan_)
      scan_sub_ = nh_.subscribe("scan", 1, &RPPControllerNode::scanCallback, this);

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_freq_), &RPPControllerNode::controlLoop, this);

    ROS_INFO("RPP Controller initialized.");
  }

private:
  void initParams()
  {
    pnh_.param("control_frequency", control_freq_, 20.0);
    pnh_.param("max_v", max_v_, 0.6);
    pnh_.param("min_v", min_v_, 0.0);
    pnh_.param("max_w", max_w_, 1.0);
    pnh_.param("min_turning_radius", min_turning_radius_, 0.5);

    pnh_.param("lookahead_time", lookahead_time_, 1.5);
    pnh_.param("min_lookahead_dist", min_lookahead_, 0.3);
    pnh_.param("max_lookahead_dist", max_lookahead_, 0.9);

    pnh_.param("regulated_min_radius", min_radius_, 1.0);
    pnh_.param("inflation_cost_factor", infl_factor_, 3.0);
    pnh_.param("scaling_dist", scaling_dist_, 0.6);
    pnh_.param("scaling_gain", scaling_gain_, 1.0);
    pnh_.param("approach_dist", approach_dist_, 0.8);
    pnh_.param("approach_min_v", approach_min_v_, 0.1);

    pnh_.param("use_scan_obstacle", use_scan_, false);
    pnh_.param("robot_radius", robot_radius_, 0.2);

    pnh_.param("goal_dist_tolerance", goal_dist_tol_, 0.15);
    pnh_.param("prune_distance", prune_distance_, 5.0);
    pnh_.param("planning_frame", planning_frame_, std::string("map"));
  }

  static double qYaw(const geometry_msgs::Quaternion& q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.empty()) return;
    std::vector<rpp_controller::Pose2D> path;
    path.reserve(msg->poses.size());
    for (const auto& ps : msg->poses)
      path.push_back({ps.pose.position.x, ps.pose.position.y, qYaw(ps.pose.orientation)});
    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_ = std::move(path);
    path_received_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_.x = msg->pose.pose.position.x;
    robot_pose_.y = msg->pose.pose.position.y;
    robot_pose_.yaw = qYaw(msg->pose.pose.orientation);
    robot_vt_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    pose_received_ = true;
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    double min_range = msg->range_max;
    for (auto r : msg->ranges)
      if (r >= msg->range_min && r < min_range) min_range = r;
    closest_obstacle_ = min_range;
  }

  std::vector<rpp_controller::Pose2D> prunePath()
  {
    std::vector<rpp_controller::Pose2D> path;
    { std::lock_guard<std::mutex> lock(path_mutex_); path = global_path_; }
    if (path.size() < 2) return path;
    double rx = robot_pose_.x, ry = robot_pose_.y;
    size_t closest = 0;
    double md = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i)
    {
      double d = std::hypot(path[i].x - rx, path[i].y - ry);
      if (d < md) { md = d; closest = i; }
    }
    std::vector<rpp_controller::Pose2D> pruned;
    double cum = 0.0;
    for (size_t i = closest; i < path.size(); ++i)
    {
      pruned.push_back(path[i]);
      if (i > closest) cum += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
      if (cum > prune_distance_) break;
    }
    return pruned;
  }

  void controlLoop(const ros::TimerEvent&)
  {
    if (!pose_received_ || !path_received_) return;

    auto pruned = prunePath();
    if (pruned.size() < 2) { publishStop(); return; }

    rpp_controller::Pose2D goal;
    { std::lock_guard<std::mutex> lock(path_mutex_); goal = global_path_.back(); }
    double dist = std::hypot(goal.x - robot_pose_.x, goal.y - robot_pose_.y);
    if (dist < goal_dist_tol_) { publishStop(); return; }

    double vt;
    { std::lock_guard<std::mutex> lock(data_mutex_); vt = robot_vt_; }

    // Obstacle cost from scan
    double obs_cost = 0.0;
    if (use_scan_)
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (closest_obstacle_ < robot_radius_ * 3.0)
        obs_cost = 254.0 * (1.0 - closest_obstacle_ / (robot_radius_ * 3.0));
    }

    auto t1 = std::chrono::steady_clock::now();
    auto cmd = controller_->compute(robot_pose_, pruned, vt, obs_cost, robot_radius_);
    auto t2 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std_msgs::Float64 solve_message; solve_message.data = ms;
    solve_time_pub_.publish(solve_message);

    ROS_INFO("[RPP] solve: %.3f ms | vx=%.3f wz=%.3f", ms, cmd.vx, cmd.vz);

    geometry_msgs::Twist twist;
    twist.linear.x = std::clamp(cmd.vx, 0.0, max_v_);
    const double yaw_limit = std::min(max_w_, twist.linear.x / min_turning_radius_);
    twist.angular.z = std::clamp(cmd.vz, -yaw_limit, yaw_limit);
    cmd_vel_pub_.publish(twist);

    local_path_pub_.publish(pose2dToPath(pruned, planning_frame_));
  }

  void publishStop() { cmd_vel_pub_.publish(geometry_msgs::Twist()); }

  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<rpp_controller::RPPController> controller_;

  double control_freq_;
  double max_v_, min_v_, max_w_, min_turning_radius_;
  double lookahead_time_, min_lookahead_, max_lookahead_;
  double min_radius_, infl_factor_, scaling_dist_, scaling_gain_;
  double approach_dist_, approach_min_v_;
  bool use_scan_;
  double robot_radius_;
  double goal_dist_tol_, prune_distance_;
  std::string planning_frame_;

  rpp_controller::Pose2D robot_pose_{};
  double robot_vt_ = 0.0;
  double closest_obstacle_ = std::numeric_limits<double>::max();
  std::vector<rpp_controller::Pose2D> global_path_;
  std::atomic<bool> pose_received_{false}, path_received_{false};
  std::mutex data_mutex_, path_mutex_, scan_mutex_;

  ros::Subscriber path_sub_, odom_sub_, scan_sub_;
  ros::Publisher cmd_vel_pub_, target_pub_, local_path_pub_, solve_time_pub_;
  ros::Timer control_timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rpp_controller_node");
  ros::NodeHandle nh, pnh("~");
  RPPControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}
