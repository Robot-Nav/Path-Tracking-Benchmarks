/**
 * @file: mpc_controller_node.cpp
 * @brief: ROS1 node for MPC path tracking controller.
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
#include <limits>
#include <memory>
#include <mutex>
#include <vector>
#include "mpc_controller/mpc_controller.hpp"

static inline nav_msgs::Path pose2dToPath(
    const std::vector<mpc_controller::Pose2D>& pp,
    const std::string& frame) {
  nav_msgs::Path m; m.header.stamp=ros::Time::now(); m.header.frame_id=frame;
  for(auto& p:pp){geometry_msgs::PoseStamped ps; ps.header=m.header;
    ps.pose.position.x=p.x;ps.pose.position.y=p.y;
    ps.pose.orientation=tf::createQuaternionMsgFromYaw(p.yaw);m.poses.push_back(ps);}
  return m;
}

class MPCControllerNode {
public:
  MPCControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    initParams();
    double dt = 1.0/control_freq_;
    controller_ = std::make_unique<mpc_controller::MPCController>(
        dt, horizon_, max_v_, min_v_, max_w_, lookahead_, acceleration_v_, acceleration_w_,
        min_turning_radius_);

    path_sub_ = nh_.subscribe("/plan", 1, &MPCControllerNode::pathCallback, this);
    odom_sub_ = nh_.subscribe("/odom1", 20, &MPCControllerNode::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    solve_time_pub_ = nh_.advertise<std_msgs::Float64>("/controller_solve_time_ms", 100);
    target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);
    control_timer_ = nh_.createTimer(ros::Duration(1.0/control_freq_), &MPCControllerNode::controlLoop, this);
    ROS_INFO("MPC Controller initialized.");
  }

private:
  void initParams() {
    pnh_.param("control_frequency", control_freq_, 20.0);
    pnh_.param("max_v", max_v_, 0.6); pnh_.param("min_v", min_v_, 0.0);
    pnh_.param("max_w", max_w_, 1.0); pnh_.param("lookahead_dist", lookahead_, 0.6);
    pnh_.param("acc_v", acceleration_v_, 1.2);
    pnh_.param("acc_w", acceleration_w_, 2.5);
    pnh_.param("min_turning_radius", min_turning_radius_, 0.5);
    pnh_.param("horizon", horizon_, 4);
    pnh_.param("goal_dist_tolerance", goal_dist_tol_, 0.15);
    pnh_.param("prune_distance", prune_distance_, 5.0);
    pnh_.param("planning_frame", planning_frame_, std::string("map"));
  }

  static double qYaw(const geometry_msgs::Quaternion& q) {
    return std::atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z));
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if(msg->poses.empty()) return;
    std::vector<mpc_controller::Pose2D> p; p.reserve(msg->poses.size());
    for(auto& ps:msg->poses) p.push_back({ps.pose.position.x, ps.pose.position.y, qYaw(ps.pose.orientation)});
    std::lock_guard<std::mutex> l(path_mutex_); global_path_=std::move(p); path_received_=true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> l(data_mutex_);
    robot_pose_.x=msg->pose.pose.position.x; robot_pose_.y=msg->pose.pose.position.y;
    robot_pose_.yaw=qYaw(msg->pose.pose.orientation);
    robot_vt_=std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    pose_received_=true;
  }

  std::vector<mpc_controller::Pose2D> prunePath() {
    std::vector<mpc_controller::Pose2D> p; {std::lock_guard<std::mutex> l(path_mutex_); p=global_path_;}
    if(p.size()<2) return p;
    double rx=robot_pose_.x, ry=robot_pose_.y; size_t closest=0; double md=std::numeric_limits<double>::max();
    for(size_t i=0;i<p.size();++i) {double d=std::hypot(p[i].x-rx,p[i].y-ry); if(d<md){md=d;closest=i;}}
    std::vector<mpc_controller::Pose2D> r; double cum=0.0;
    for(size_t i=closest;i<p.size();++i) {
      r.push_back(p[i]); if(i>closest) cum+=std::hypot(p[i].x-p[i-1].x,p[i].y-p[i-1].y);
      if(cum>prune_distance_) break;
    } return r;
  }

  void controlLoop(const ros::TimerEvent&) {
    if(!pose_received_||!path_received_) return;
    auto pruned=prunePath(); if(pruned.size()<2){publishStop();return;}
    mpc_controller::Pose2D goal;
    {std::lock_guard<std::mutex> l(path_mutex_); goal=global_path_.back();}
    if(std::hypot(goal.x-robot_pose_.x,goal.y-robot_pose_.y)<goal_dist_tol_){publishStop();return;}
    double vt; {std::lock_guard<std::mutex> l(data_mutex_); vt=robot_vt_;}

    auto t1 = std::chrono::steady_clock::now();
    auto cmd=controller_->compute(robot_pose_,pruned,vt);
    auto t2 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std_msgs::Float64 solve_message; solve_message.data = ms;
    solve_time_pub_.publish(solve_message);

    ROS_INFO("[MPC] solve: %.3f ms | vx=%.3f wz=%.3f", ms, cmd.vx, cmd.vz);

    geometry_msgs::Twist tw;
    tw.linear.x=std::clamp(cmd.vx, 0.0, max_v_);
    const double yaw_limit=std::min(max_w_, tw.linear.x/min_turning_radius_);
    tw.angular.z=std::clamp(cmd.vz, -yaw_limit, yaw_limit);
    cmd_vel_pub_.publish(tw);
    local_path_pub_.publish(pose2dToPath(pruned, planning_frame_));
  }

  void publishStop() { cmd_vel_pub_.publish(geometry_msgs::Twist()); }

  ros::NodeHandle nh_,pnh_; std::unique_ptr<mpc_controller::MPCController> controller_;
  double control_freq_,max_v_,min_v_,max_w_,lookahead_,acceleration_v_,acceleration_w_;
  double min_turning_radius_,goal_dist_tol_,prune_distance_;
  int horizon_; std::string planning_frame_;
  mpc_controller::Pose2D robot_pose_{}; double robot_vt_;
  std::vector<mpc_controller::Pose2D> global_path_;
  std::atomic<bool> pose_received_{false}, path_received_{false};
  std::mutex data_mutex_, path_mutex_;
  ros::Subscriber path_sub_,odom_sub_; ros::Publisher cmd_vel_pub_,target_pub_,local_path_pub_,solve_time_pub_; ros::Timer control_timer_;
};

int main(int argc,char** argv) {
  ros::init(argc,argv,"mpc_controller_node"); ros::NodeHandle nh,pnh("~");
  MPCControllerNode node(nh,pnh); ros::spin(); return 0;
}
