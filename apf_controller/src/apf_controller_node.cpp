#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "apf_controller/apf_controller.hpp"

namespace
{

nav_msgs::Path pose2dToPath(
  const std::vector<apf_controller::Pose2D> & path, const std::string & frame)
{
  nav_msgs::Path message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = frame;
  message.poses.reserve(path.size());
  for (const auto & point : path) {
    geometry_msgs::PoseStamped pose;
    pose.header = message.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(point.yaw);
    message.poses.push_back(pose);
  }
  return message;
}

double quaternionYaw(const geometry_msgs::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool validQuaternion(const geometry_msgs::Quaternion & q)
{
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(norm) && norm > 0.25;
}

}  // namespace

class APFControllerNode
{
public:
  APFControllerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : nh_(nh), pnh_(pnh)
  {
    initParams();
    controller_ = std::make_unique<apf_controller::APFController>(
      max_v_, min_v_, max_w_, lookahead_, heading_gain_, smooth_window_);
    path_sub_ = nh_.subscribe("/plan", 1, &APFControllerNode::pathCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &APFControllerNode::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    solve_time_pub_ = nh_.advertise<std_msgs::Float64>("/controller_solve_time_ms", 100);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);
    timer_ = nh_.createTimer(
      ros::Duration(1.0 / control_frequency_), &APFControllerNode::controlLoop, this);
    ROS_INFO("APF pure-tracking controller initialized; odom=%s", odom_topic_.c_str());
  }

private:
  void initParams()
  {
    pnh_.param("control_frequency", control_frequency_, 20.0);
    pnh_.param("max_v", max_v_, 0.6);
    pnh_.param("min_v", min_v_, 0.0);
    pnh_.param("max_w", max_w_, 1.0);
    pnh_.param("min_turning_radius", min_turning_radius_, 0.5);
    pnh_.param("lookahead", lookahead_, 0.6);
    pnh_.param("heading_gain", heading_gain_, 2.0);
    pnh_.param("smooth_window", smooth_window_, 3);
    pnh_.param("goal_dist_tolerance", goal_dist_tolerance_, 0.15);
    pnh_.param("prune_distance", prune_distance_, 5.0);
    pnh_.param("odom_timeout", odom_timeout_, 0.30);
    pnh_.param("planning_frame", planning_frame_, std::string("map"));
    pnh_.param("odom_topic", odom_topic_, std::string("/odom1"));
    control_frequency_ = std::max(1.0, control_frequency_);
  }

  void pathCallback(const nav_msgs::Path::ConstPtr & message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != planning_frame_) {
      ROS_ERROR_THROTTLE(
        1.0, "APF rejected path frame '%s'; expected '%s'.",
        message->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }
    if (message->poses.size() < 2) {
      {
        std::lock_guard<std::mutex> lock(path_mutex_);
        global_path_.clear();
      }
      path_received_ = false;
      controller_->reset();
      publishStop();
      return;
    }

    std::vector<apf_controller::Pose2D> path;
    path.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
      if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) continue;
      path.push_back({
        pose.pose.position.x, pose.pose.position.y,
        validQuaternion(pose.pose.orientation) ? quaternionYaw(pose.pose.orientation) : 0.0});
    }
    if (path.size() < 2) {
      ROS_WARN_THROTTLE(1.0, "APF rejected path with fewer than two valid poses.");
      publishStop();
      return;
    }
    for (size_t i = 0; i < path.size(); ++i) {
      const size_t previous = i > 0 ? i - 1 : i;
      const size_t next = i + 1 < path.size() ? i + 1 : i;
      path[i].yaw = std::atan2(
        path[next].y - path[previous].y, path[next].x - path[previous].x);
    }

    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_ = std::move(path);
    path_received_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr & message)
  {
    if (!std::isfinite(message->pose.pose.position.x) ||
      !std::isfinite(message->pose.pose.position.y) ||
      !validQuaternion(message->pose.pose.orientation))
    {
      ROS_ERROR_THROTTLE(1.0, "APF rejected invalid odometry sample.");
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_.x = message->pose.pose.position.x;
    robot_pose_.y = message->pose.pose.position.y;
    robot_pose_.yaw = quaternionYaw(message->pose.pose.orientation);
    last_odom_time_ = ros::Time::now();
    odom_received_ = true;
  }

  std::vector<apf_controller::Pose2D> prunePath(
    const apf_controller::Pose2D & robot, apf_controller::Pose2D & goal) const
  {
    std::vector<apf_controller::Pose2D> path;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      path = global_path_;
    }
    if (path.size() < 2) return path;
    goal = path.back();

    size_t closest_index = 0;
    double minimum_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
      const double distance = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
      if (distance < minimum_distance) {
        minimum_distance = distance;
        closest_index = i;
      }
    }

    std::vector<apf_controller::Pose2D> result;
    double accumulated = 0.0;
    for (size_t i = closest_index; i < path.size(); ++i) {
      result.push_back(path[i]);
      if (i > closest_index) {
        accumulated += std::hypot(
          path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      }
      if (accumulated >= prune_distance_) break;
    }
    return result;
  }

  void publishStop()
  {
    cmd_vel_pub_.publish(geometry_msgs::Twist{});
  }

  void controlLoop(const ros::TimerEvent &)
  {
    if (!odom_received_ || !path_received_) return;

    apf_controller::Pose2D robot;
    ros::Time odom_time;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      robot = robot_pose_;
      odom_time = last_odom_time_;
    }
    const double odom_age = (ros::Time::now() - odom_time).toSec();
    if (odom_age > odom_timeout_) {
      ROS_ERROR_THROTTLE(1.0, "APF odometry timeout (%.3f s); stopping.", odom_age);
      controller_->reset();
      publishStop();
      return;
    }

    apf_controller::Pose2D goal;
    const auto path = prunePath(robot, goal);
    if (path.size() < 2 ||
      std::hypot(goal.x - robot.x, goal.y - robot.y) < goal_dist_tolerance_)
    {
      controller_->reset();
      publishStop();
      return;
    }

    const auto start = std::chrono::steady_clock::now();
    const auto command = controller_->compute(robot, path);
    const auto finish = std::chrono::steady_clock::now();
    const double solve_ms =
      std::chrono::duration<double, std::milli>(finish - start).count();
    std_msgs::Float64 solve_message;
    solve_message.data = solve_ms;
    solve_time_pub_.publish(solve_message);

    geometry_msgs::Twist message;
    message.linear.x = std::clamp(command.vx, 0.0, max_v_);
    const double yaw_limit = std::min(max_w_, message.linear.x / min_turning_radius_);
    message.angular.z = std::clamp(command.vz, -yaw_limit, yaw_limit);
    cmd_vel_pub_.publish(message);
    local_path_pub_.publish(pose2dToPath(path, planning_frame_));
    ROS_INFO_THROTTLE(
      1.0, "[APF tracking] solve=%.3fms, vx=%.3f, wz=%.3f",
      solve_ms, command.vx, command.vz);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<apf_controller::APFController> controller_;
  double control_frequency_{20.0};
  double max_v_{0.6};
  double min_v_{0.0};
  double max_w_{1.0};
  double min_turning_radius_{0.5};
  double lookahead_{0.6};
  double heading_gain_{2.0};
  int smooth_window_{3};
  double goal_dist_tolerance_{0.15};
  double prune_distance_{5.0};
  double odom_timeout_{0.30};
  std::string planning_frame_{"map"};
  std::string odom_topic_{"/odom1"};
  apf_controller::Pose2D robot_pose_{};
  std::vector<apf_controller::Pose2D> global_path_;
  std::atomic<bool> odom_received_{false};
  std::atomic<bool> path_received_{false};
  ros::Time last_odom_time_;
  mutable std::mutex data_mutex_;
  mutable std::mutex path_mutex_;
  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher solve_time_pub_;
  ros::Publisher local_path_pub_;
  ros::Timer timer_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "apf_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  APFControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}
