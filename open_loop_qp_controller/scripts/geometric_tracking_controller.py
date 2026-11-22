#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Track a path using odometry, fixed lookahead, and smoothed commands."""

import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64


EPS = 1.0e-6


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class GeometricPathTracker:
    """Project the robot onto the plan and steer toward a point ahead."""

    def __init__(self):
        rospy.init_node("geometric_tracking_controller")
        self.lock = threading.Lock()
        self.dt = max(0.01, float(rospy.get_param("~control_period", 0.05)))
        self.lookahead_distance = max(
            0.05, float(rospy.get_param("~lookahead_distance", 1.5))
        )
        self.cruise_speed = max(0.0, float(rospy.get_param("~cruise_speed", 0.60)))
        self.max_speed = max(
            self.cruise_speed, float(rospy.get_param("~max_speed", 0.70))
        )
        self.max_angular_speed = max(
            0.01, float(rospy.get_param("~max_angular_speed", 1.0))
        )
        self.max_acceleration = max(
            0.01, float(rospy.get_param("~max_acceleration", 1.2))
        )
        self.max_deceleration = max(
            0.01, float(rospy.get_param("~max_deceleration", 1.2))
        )
        self.max_angular_acceleration = max(
            0.01, float(rospy.get_param("~max_angular_acceleration", 2.5))
        )
        self.max_jerk = max(0.01, float(rospy.get_param("~max_jerk", 4.0)))
        self.max_lateral_acceleration = max(
            0.01, float(rospy.get_param("~max_lateral_acceleration", 0.60))
        )
        self.min_turning_radius = max(
            0.01, float(rospy.get_param("~min_turning_radius", 0.50))
        )
        self.yaw_filter_alpha = clamp(
            float(rospy.get_param("~yaw_filter_alpha", 0.8)), 0.0, 0.99
        )
        self.goal_tolerance = max(
            0.01, float(rospy.get_param("~goal_tolerance", 0.05))
        )
        self.odom_timeout = max(
            self.dt, float(rospy.get_param("~odom_timeout", 0.30))
        )

        self.path = []
        self.path_frame = ""
        self.arc = []
        self.path_signature = None
        self.robot = None
        self.odom_frame = ""
        self.odom_stamp = rospy.Time(0)
        self.filtered_yaw = 0.0
        self.have_filtered_yaw = False
        self.current_speed = 0.0
        self.current_acceleration = 0.0
        self.last_angular_speed = 0.0

        self.command_publisher = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", "/cmd_vel"),
            Twist,
            queue_size=1,
        )
        self.local_path_publisher = rospy.Publisher(
            "/local_path", Path, queue_size=1
        )
        self.solve_time_publisher = rospy.Publisher(
            "/controller_solve_time_ms", Float64, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param("~plan_topic", "/plan"), Path, self.path_callback, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/odom1"), Odometry, self.odom_callback, queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_callback)
        rospy.on_shutdown(self.publish_stop)

    def path_callback(self, message):
        points = []
        for pose in message.poses:
            point = (float(pose.pose.position.x), float(pose.pose.position.y))
            if not all(math.isfinite(value) for value in point):
                rospy.logwarn_throttle(1.0, "Geometric tracker requires finite waypoints")
                return
            if not points or math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) > EPS:
                points.append(point)
        if len(points) < 2:
            rospy.logwarn_throttle(1.0, "Geometric tracker requires two distinct waypoints")
            return
        arc = [0.0]
        for index in range(1, len(points)):
            arc.append(arc[-1] + math.hypot(
                points[index][0] - points[index - 1][0],
                points[index][1] - points[index - 1][1],
            ))
        with self.lock:
            signature = tuple(
                (round(point[0], 4), round(point[1], 4)) for point in points
            )
            path_changed = signature != self.path_signature or self.path_frame != message.header.frame_id
            self.path = points
            self.path_frame = message.header.frame_id
            self.arc = arc
            self.path_signature = signature
            if path_changed:
                self._reset_motion()

    def odom_callback(self, message):
        pose = message.pose.pose
        x, y, yaw = pose.position.x, pose.position.y, quaternion_yaw(pose.orientation)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        with self.lock:
            frame_changed = self.odom_frame != message.header.frame_id
            if not self.have_filtered_yaw or frame_changed:
                self.filtered_yaw = yaw
                self.have_filtered_yaw = True
            else:
                self.filtered_yaw = wrap_angle(
                    self.filtered_yaw + (1.0 - self.yaw_filter_alpha)
                    * wrap_angle(yaw - self.filtered_yaw)
                )
            self.robot = (x, y, self.filtered_yaw)
            self.odom_frame = message.header.frame_id
            self.odom_stamp = rospy.Time.now()

    def _reset_motion(self):
        self.current_speed = 0.0
        self.current_acceleration = 0.0
        self.last_angular_speed = 0.0

    def _project(self, x, y):
        best = None
        for index in range(len(self.path) - 1):
            x0, y0 = self.path[index]
            x1, y1 = self.path[index + 1]
            dx, dy = x1 - x0, y1 - y0
            length_sq = dx * dx + dy * dy
            if length_sq <= EPS:
                continue
            ratio = clamp(((x - x0) * dx + (y - y0) * dy) / length_sq, 0.0, 1.0)
            px, py = x0 + ratio * dx, y0 + ratio * dy
            distance_sq = (x - px) ** 2 + (y - py) ** 2
            if best is None or distance_sq < best[0]:
                length = math.sqrt(length_sq)
                best = (distance_sq, self.arc[index] + ratio * length, index, ratio)
        return best

    def _sample(self, s_value):
        s_value = clamp(s_value, 0.0, self.arc[-1])
        for index in range(len(self.arc) - 1):
            if s_value <= self.arc[index + 1] + EPS:
                segment = max(EPS, self.arc[index + 1] - self.arc[index])
                ratio = clamp((s_value - self.arc[index]) / segment, 0.0, 1.0)
                x0, y0 = self.path[index]
                x1, y1 = self.path[index + 1]
                return x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)
        return self.path[-1]

    def _smooth_speed(self, target_speed):
        # Ramp acceleration before integrating it into the speed command.
        speed_error = target_speed - self.current_speed
        desired_acceleration = clamp(
            speed_error / self.dt, -self.max_deceleration, self.max_acceleration
        )
        acceleration_step = self.max_jerk * self.dt
        self.current_acceleration = clamp(
            desired_acceleration,
            self.current_acceleration - acceleration_step,
            self.current_acceleration + acceleration_step,
        )
        next_speed = self.current_speed + self.current_acceleration * self.dt
        if (speed_error >= 0.0 and next_speed >= target_speed) or (
            speed_error < 0.0 and next_speed <= target_speed
        ):
            next_speed = target_speed
            self.current_acceleration = 0.0
        self.current_speed = clamp(next_speed, 0.0, self.max_speed)
        return self.current_speed

    def _command(self):
        if self.robot is None or len(self.path) < 2:
            return 0.0, 0.0, None
        x, y, yaw = self.robot
        projection = self._project(x, y)
        if projection is None:
            return 0.0, 0.0, None
        _, progress, _, _ = projection
        goal_x, goal_y = self.path[-1]
        goal_distance = math.hypot(goal_x - x, goal_y - y)
        if goal_distance <= self.goal_tolerance:
            self._reset_motion()
            return 0.0, 0.0, progress

        target_x, target_y = self._sample(progress + self.lookahead_distance)
        target_heading = math.atan2(target_y - y, target_x - x)
        heading_error = wrap_angle(target_heading - yaw)
        curvature = clamp(
            2.0 * math.sin(heading_error) / self.lookahead_distance,
            -1.0 / self.min_turning_radius,
            1.0 / self.min_turning_radius,
        )
        speed_limit = self.cruise_speed
        if abs(curvature) > EPS:
            speed_limit = min(
                speed_limit,
                self.max_angular_speed / abs(curvature),
                math.sqrt(self.max_lateral_acceleration / abs(curvature)),
            )
        # Slow down for heading error and taper speed smoothly near the goal.
        speed_limit *= 1.0 - 0.4 * abs(heading_error) / math.pi
        if goal_distance < self.lookahead_distance:
            ratio = clamp(goal_distance / self.lookahead_distance, 0.0, 1.0)
            speed_limit *= ratio * ratio * (3.0 - 2.0 * ratio)
        speed = self._smooth_speed(clamp(speed_limit, 0.0, self.max_speed))
        desired_angular_speed = clamp(
            speed * curvature, -self.max_angular_speed, self.max_angular_speed
        )
        angular_step = self.max_angular_acceleration * self.dt
        angular_speed = clamp(
            desired_angular_speed,
            self.last_angular_speed - angular_step,
            self.last_angular_speed + angular_step,
        )
        self.last_angular_speed = angular_speed
        return speed, angular_speed, progress

    def control_callback(self, _event):
        started = time.perf_counter()
        with self.lock:
            odom_age = (rospy.Time.now() - self.odom_stamp).to_sec()
            odom_fresh = not self.odom_stamp.is_zero() and 0.0 <= odom_age <= self.odom_timeout
            same_frame = not self.path_frame or not self.odom_frame or self.path_frame == self.odom_frame
            if odom_fresh and same_frame:
                speed, angular_speed, progress = self._command()
            else:
                self._reset_motion()
                speed, angular_speed, progress = 0.0, 0.0, None
            path_snapshot = self.path
            arc_snapshot = self.arc
            path_frame = self.path_frame or self.odom_frame or "map"
        command = Twist()
        command.linear.x = speed
        command.angular.z = angular_speed
        self.command_publisher.publish(command)
        self.solve_time_publisher.publish(Float64(1000.0 * (time.perf_counter() - started)))
        if progress is not None:
            local_path = Path()
            local_path.header.frame_id = path_frame
            local_path.header.stamp = rospy.Time.now()
            for point, s_value in zip(path_snapshot, arc_snapshot):
                if s_value + EPS < progress:
                    continue
                pose = PoseStamped()
                pose.header = local_path.header
                pose.pose.position.x, pose.pose.position.y = point
                pose.pose.orientation.w = 1.0
                local_path.poses.append(pose)
            self.local_path_publisher.publish(local_path)

    def publish_stop(self):
        self.command_publisher.publish(Twist())


if __name__ == "__main__":
    GeometricPathTracker()
    rospy.spin()
