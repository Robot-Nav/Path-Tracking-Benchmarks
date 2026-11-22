/**
 * @file: pid_controller_node.cpp
 * @brief: ROS1 node for PID path tracking controller.
 *
 * Subscribes to /plan (nav_msgs/Path) and /odom1 (nav_msgs/Odometry),
 * publishes /cmd_vel (geometry_msgs/Twist), /local_path (nav_msgs/Path).
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

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

#include "pid_controller/pid_controller.hpp"

static inline nav_msgs::Path pose2dToPath(
    const std::vector<pid_controller::Pose2D>& pp,
    const std::string& frame) {
  nav_msgs::Path m; m.header.stamp=ros::Time::now(); m.header.frame_id=frame;
  for(auto& p:pp){geometry_msgs::PoseStamped ps; ps.header=m.header;
    ps.pose.position.x=p.position.x;ps.pose.position.y=p.position.y;
    ps.pose.orientation=tf::createQuaternionMsgFromYaw(p.yaw);m.poses.push_back(ps);}
  return m;
}

class PIDControllerNode
{
public:
  PIDControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
  {
    initParams();
    controller_ = std::make_unique<pid_controller::PIDController>(
        max_v_, min_v_, max_w_, k_, L_, lookahead_dist_);

    path_sub_  = nh_.subscribe("/plan", 1, &PIDControllerNode::pathCallback, this);
    odom_sub_  = nh_.subscribe("/odom1", 20, &PIDControllerNode::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    solve_time_pub_ = nh_.advertise<std_msgs::Float64>("/controller_solve_time_ms", 100);
    target_pub_  = nh_.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_freq_), &PIDControllerNode::controlLoop, this);

    ROS_INFO("PID Controller initialized.");
  }

private:
  void initParams()
  {
    pnh_.param("control_frequency", control_freq_, 20.0);
    pnh_.param("max_v", max_v_, 0.6);
    pnh_.param("min_v", min_v_, 0.0);
    pnh_.param("max_w", max_w_, 1.0);
    pnh_.param("min_turning_radius", min_turning_radius_, 0.5);
    pnh_.param("k", k_, 1.0);
    pnh_.param("L", L_, 0.2);
    pnh_.param("lookahead_dist", lookahead_dist_, 0.6);
    pnh_.param("goal_dist_tolerance", goal_dist_tol_, 0.15);
    pnh_.param("prune_distance", prune_distance_, 5.0);
    pnh_.param("planning_frame", planning_frame_, std::string("map"));
    pnh_.param("strict_frame_check", strict_frame_check_, true);
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
      ROS_ERROR_THROTTLE(1.0, "PID: rejected path in frame '%s'", msg->header.frame_id.c_str());
      return;
    }
    std::vector<pid_controller::Pose2D> path;
    path.reserve(msg->poses.size());
    for (const auto& ps : msg->poses)
      path.push_back({{ps.pose.position.x, ps.pose.position.y}, quaternionYaw(ps.pose.orientation)});

    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_ = std::move(path);
    path_received_ = true;
    ROS_INFO("PID: Path accepted (%zu points).", global_path_.size());
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if (strict_frame_check_ && !msg->header.frame_id.empty() &&
        msg->header.frame_id != planning_frame_)
      return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_.position.x = msg->pose.pose.position.x;
    robot_pose_.position.y = msg->pose.pose.position.y;
    robot_pose_.yaw = quaternionYaw(msg->pose.pose.orientation);
    pose_received_ = true;
  }

  std::vector<pid_controller::Pose2D> prunePath()
  {
    std::vector<pid_controller::Pose2D> path;
    { std::lock_guard<std::mutex> lock(path_mutex_); path = global_path_; }
    if (path.size() < 2) return path;

    double rx = robot_pose_.position.x, ry = robot_pose_.position.y;
    size_t closest = 0;
    double min_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i)
    {
      double d = std::hypot(path[i].position.x - rx, path[i].position.y - ry);
      if (d < min_d) { min_d = d; closest = i; }
    }
    std::vector<pid_controller::Pose2D> pruned;
    double cum = 0.0;
    for (size_t i = closest; i < path.size(); ++i)
    {
      pruned.push_back(path[i]);
      if (i > closest)
        cum += std::hypot(path[i].position.x - path[i-1].position.x,
                          path[i].position.y - path[i-1].position.y);
      if (cum > prune_distance_) break;
    }
    return pruned;
  }

  void controlLoop(const ros::TimerEvent&)
  {
    if (!pose_received_ || !path_received_) return;

    auto pruned = prunePath();
    if (pruned.size() < 2) { publishStop(); return; }

    pid_controller::Pose2D goal;
    { std::lock_guard<std::mutex> lock(path_mutex_); goal = global_path_.back(); }
    double dist = std::hypot(goal.position.x - robot_pose_.position.x,
                              goal.position.y - robot_pose_.position.y);
    if (dist < goal_dist_tol_) { publishStop(); return; }

    auto t1 = std::chrono::steady_clock::now();
    pid_controller::Twist2D cmd = controller_->compute(robot_pose_, pruned);
    auto t2 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std_msgs::Float64 solve_message; solve_message.data = ms;
    solve_time_pub_.publish(solve_message);

    ROS_INFO("[PID] solve: %.3f ms | vx=%.3f wz=%.3f", ms, cmd.vx, cmd.vz);

    geometry_msgs::Twist twist;
    twist.linear.x = std::clamp(cmd.vx, 0.0, max_v_);
    const double yaw_limit = std::min(max_w_, twist.linear.x / min_turning_radius_);
    twist.angular.z = std::clamp(cmd.vz, -yaw_limit, yaw_limit);
    cmd_vel_pub_.publish(twist);

    local_path_pub_.publish(pose2dToPath(pruned, planning_frame_));
  }

  void publishStop() { cmd_vel_pub_.publish(geometry_msgs::Twist()); }

  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<pid_controller::PIDController> controller_;

  double control_freq_;
  double max_v_, min_v_, max_w_, min_turning_radius_, k_, L_, lookahead_dist_;
  double goal_dist_tol_, prune_distance_;
  std::string planning_frame_;
  bool strict_frame_check_;

  pid_controller::Pose2D robot_pose_{};
  std::vector<pid_controller::Pose2D> global_path_;
  std::atomic<bool> pose_received_{false}, path_received_{false};
  std::mutex data_mutex_, path_mutex_;

  ros::Subscriber path_sub_, odom_sub_;
  ros::Publisher cmd_vel_pub_, target_pub_, local_path_pub_, solve_time_pub_;
  ros::Timer control_timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_controller_node");
  ros::NodeHandle nh, pnh("~");
  PIDControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}
