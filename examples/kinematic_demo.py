#!/usr/bin/env python3
"""Publish a synthetic Path and simulate odometry from Twist commands."""

import argparse
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path


def pose_at(x, y, yaw):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def make_path(shape, frame):
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = rospy.Time.now()
    length = 5.0
    amplitude = 0.35 if shape == "s-curve" else 0.0
    wave_number = 2.0 * math.pi / length
    for index in range(101):
        x = length * index / 100.0
        pose = pose_at(x, amplitude * math.sin(wave_number * x),
                       math.atan(amplitude * wave_number * math.cos(wave_number * x)))
        pose.header = path.header
        path.poses.append(pose)
    return path


class KinematicDemo:
    def __init__(self, shape):
        path = make_path(shape, "map")
        orientation = path.poses[0].pose.orientation
        self.x = self.y = 0.0
        self.yaw = 2.0 * math.atan2(orientation.z, orientation.w)
        self.command = Twist()
        self.command_time = None
        self.state_lock = threading.Lock()
        self.frame = path.header.frame_id

        self.odom_pub = rospy.Publisher("/odom1", Odometry, queue_size=10)
        self.plan_pub = rospy.Publisher("/plan", Path, queue_size=1, latch=True)
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.receive_command, queue_size=1)
        self.plan_pub.publish(path)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.step)

    def receive_command(self, message):
        with self.state_lock:
            self.command = message
            self.command_time = rospy.Time.now()

    def step(self, event):
        now = event.current_real
        dt = (now - event.last_real).to_sec() if event.last_real is not None else 0.0
        with self.state_lock:
            fresh = self.command_time is not None and (now - self.command_time).to_sec() < 0.5
            v = self.command.linear.x if fresh else 0.0
            omega = self.command.angular.z if fresh else 0.0

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += omega * dt

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame
        odom.child_frame_id = "base_link"
        odom.pose.pose = pose_at(self.x, self.y, self.yaw).pose
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", choices=("straight", "s-curve"), default="straight")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("tracking_kinematic_demo")
    demo = KinematicDemo(args.path)
    rospy.spin()
    demo.timer.shutdown()


if __name__ == "__main__":
    main()
