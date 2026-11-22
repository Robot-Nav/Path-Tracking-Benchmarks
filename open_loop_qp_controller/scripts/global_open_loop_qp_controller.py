#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Plan and play a velocity sequence for the first valid path."""

import math
import threading
import time

import numpy as np
import rospy
import scipy.sparse as sparse
import osqp
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float64


EPS = 1.0e-6


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class GlobalOpenLoopQPController:
    """Fit waypoint speeds once and play them without pose feedback."""

    def __init__(self):
        rospy.init_node("open_loop_qp_controller")
        self.control_period = max(0.01, float(rospy.get_param("~control_period", 0.05)))
        self.cruise_speed = max(0.01, float(rospy.get_param("~cruise_speed", 0.60)))
        self.max_speed = max(self.cruise_speed, float(rospy.get_param("~max_speed", 0.70)))
        self.max_acceleration = max(0.01, float(rospy.get_param("~max_acceleration", 1.2)))
        self.max_deceleration = max(0.01, float(rospy.get_param("~max_deceleration", 1.2)))
        self.max_angular_speed = max(0.01, float(rospy.get_param("~max_angular_speed", 1.0)))
        self.max_lateral_acceleration = max(0.01, float(rospy.get_param("~max_lateral_acceleration", 0.60)))
        self.min_turning_radius = max(0.01, float(rospy.get_param("~min_turning_radius", 0.50)))
        self.speed_tracking_weight = max(0.01, float(rospy.get_param("~speed_tracking_weight", 12.0)))
        self.speed_smoothness_weight = max(0.0, float(rospy.get_param("~speed_smoothness_weight", 4.0)))
        self.plan_topic = rospy.get_param("~plan_topic", "/plan")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.command_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.local_path_publisher = rospy.Publisher("/local_path", Path, queue_size=1, latch=True)
        self.solve_time_publisher = rospy.Publisher("/controller_solve_time_ms", Float64, queue_size=1, latch=True)
        self.started = False
        self.sequence = []
        self.start_time = None
        self.plan_lock = threading.Lock()
        rospy.Subscriber(self.plan_topic, Path, self.path_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.control_period), self.timer_callback)
        rospy.on_shutdown(self.publish_stop)

    @staticmethod
    def _distinct_points(message):
        points = []
        for pose in message.poses:
            point = (float(pose.pose.position.x), float(pose.pose.position.y))
            if not all(math.isfinite(value) for value in point):
                return []
            if not points or math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) > EPS:
                points.append(point)
        return points

    def _curvature_profile(self, points):
        count = len(points)
        headings = []
        lengths = []
        for index in range(count - 1):
            dx = points[index + 1][0] - points[index][0]
            dy = points[index + 1][1] - points[index][1]
            lengths.append(max(math.hypot(dx, dy), EPS))
            headings.append(math.atan2(dy, dx))
        curvature = np.zeros(count, dtype=float)
        for index in range(1, count - 1):
            ds = 0.5 * (lengths[index - 1] + lengths[index])
            curvature[index] = clamp(
                wrap_angle(headings[index] - headings[index - 1]) / max(ds, EPS),
                -1.0 / self.min_turning_radius,
                1.0 / self.min_turning_radius,
            )
        curvature[0] = curvature[1] if count > 2 else 0.0
        curvature[-1] = curvature[-2] if count > 2 else 0.0
        return np.asarray(lengths, dtype=float), curvature

    def _solve_speed_qp(self, points, segment_lengths, curvature):
        count = len(points)
        curvature_abs = np.maximum(np.abs(curvature), EPS)
        curvature_limit = np.minimum(
            self.max_angular_speed / curvature_abs,
            np.sqrt(self.max_lateral_acceleration / curvature_abs),
        )
        upper = np.minimum(self.max_speed, curvature_limit)
        upper[0] = 0.0
        upper[-1] = 0.0
        reference = np.minimum(self.cruise_speed, upper)

        identity = sparse.eye(count, format="csc")
        difference = sparse.diags(
            [-np.ones(count - 1), np.ones(count - 1)], [0, 1],
            shape=(count - 1, count), format="csc",
        )
        quadratic = 2.0 * (
            self.speed_tracking_weight * identity
            + self.speed_smoothness_weight * (difference.T @ difference)
        )
        linear = -2.0 * self.speed_tracking_weight * reference

        # Approximate the interval between adjacent waypoints at cruise speed.
        intervals = segment_lengths / max(self.cruise_speed, 0.05)
        up_step = self.max_acceleration * intervals
        down_step = self.max_deceleration * intervals
        constraints = sparse.vstack([identity, difference], format="csc")
        lower = np.concatenate([np.zeros(count), -down_step])
        upper_bound = np.concatenate([upper, up_step])

        solver = osqp.OSQP()
        solver.setup(P=sparse.triu(quadratic).tocsc(), q=linear, A=constraints,
                     l=lower, u=upper_bound, verbose=False, polishing=True,
                     eps_abs=1.0e-4, eps_rel=1.0e-4)
        result = solver.solve()
        if result.info.status_val not in (1, 2) or result.x is None:
            rospy.logwarn("Global open-loop QP failed (%s); waiting for another path", result.info.status)
            return None
        return np.clip(result.x, 0.0, upper)

    def path_callback(self, message):
        with self.plan_lock:
            if not self.started:
                self.prepare_sequence(message)

    def prepare_sequence(self, message):
        points = self._distinct_points(message)
        if len(points) < 2:
            rospy.logwarn_throttle(1.0, "Open-loop QP requires two distinct, finite waypoints")
            return
        if len(points) == 2:
            start, goal = points
            midpoint = ((start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0)
            points.insert(1, midpoint)
        started = time.perf_counter()
        segment_lengths, curvature = self._curvature_profile(points)
        speeds = self._solve_speed_qp(points, segment_lengths, curvature)
        if speeds is None:
            self.publish_stop()
            return
        sequence = []
        for index in range(len(points) - 1):
            segment_speed = max(0.01, 0.5 * (speeds[index] + speeds[index + 1]))
            duration = segment_lengths[index] / segment_speed
            steps = max(1, int(math.ceil(duration / self.control_period)))
            for step in range(steps):
                ratio = float(step + 1) / steps
                speed = (1.0 - ratio) * speeds[index] + ratio * speeds[index + 1]
                omega = clamp(speed * curvature[index], -self.max_angular_speed, self.max_angular_speed)
                sequence.append((float(speed), float(omega)))
        self.solve_time_publisher.publish(Float64(1000.0 * (time.perf_counter() - started)))
        self.local_path_publisher.publish(message)
        self.sequence = sequence
        self.start_time = rospy.Time.now()
        self.started = True
        rospy.loginfo("global open-loop QP prepared %d commands from %d plan points", len(sequence), len(points))

    def timer_callback(self, _event):
        if not self.started or self.start_time is None:
            return
        index = int((rospy.Time.now() - self.start_time).to_sec() / self.control_period)
        command = Twist()
        if 0 <= index < len(self.sequence):
            command.linear.x, command.angular.z = self.sequence[index]
        self.command_publisher.publish(command)

    def publish_stop(self):
        self.command_publisher.publish(Twist())


if __name__ == "__main__":
    GlobalOpenLoopQPController()
    rospy.spin()
