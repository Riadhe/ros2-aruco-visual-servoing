"""Node 3: closed-loop rotation on the robot heading (yaw), as an action server.

Goal: rotate by a relative angle (rad, positive = left / counter-clockwise).
The yaw is read from odometry (EKF output by default), never timed.
"""
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from erl1_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Yaw (rotation around z) from a quaternion: formula from the assignment slide."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def normalize_angle(angle):
    """Wrap any angle into [-pi, pi]: formula from the assignment slide."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Rotation(Node):

    def __init__(self):
        super().__init__('rotation')

        params = {
            'odom_topic': '/odometry/filtered',   # EKF output (or /odom)
            'cmd_vel_topic': '/cmd_vel',
            'k_yaw': 1.5,             # proportional gain [rad/s per rad]
            'max_omega': 0.8,         # [rad/s]
            'min_omega': 0.15,        # avoid stalling when the error gets small [rad/s]
            'yaw_tolerance': 0.02,    # ~1.1 degrees [rad]
            'odom_timeout': 2.0,      # abort if no odometry for this long [s]
            'rate': 30.0,             # control loop frequency [Hz]
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in params}
        self.k_yaw = p['k_yaw']
        self.max_omega, self.min_omega = p['max_omega'], p['min_omega']
        self.yaw_tol = p['yaw_tolerance']
        self.odom_timeout = p['odom_timeout']
        self.period = 1.0 / p['rate']

        self.yaw = None
        self.yaw_time = 0.0

        group = ReentrantCallbackGroup()
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, 10,
                                 callback_group=group)
        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)

        self.server = ActionServer(
            self, Rotate, 'rotate',
            execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info(f"Rotation ready (action: /rotate, odom: {p['odom_topic']})")

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.yaw_time = time.monotonic()

    def odom_ok(self):
        return self.yaw is not None and time.monotonic() - self.yaw_time < self.odom_timeout

    def stop(self):
        self.cmd_pub.publish(Twist())

    def execute(self, goal_handle):
        result = Rotate.Result()
        feedback = Rotate.Feedback()
        angle = goal_handle.request.angle

        if not self.odom_ok():
            self.get_logger().error('No odometry: cannot rotate')
            goal_handle.abort()
            result.success, result.final_error = False, float('nan')
            return result

        target = normalize_angle(self.yaw + angle)
        self.get_logger().info(
            f'Rotate {math.degrees(angle):.0f} deg: yaw {math.degrees(self.yaw):.1f}'
            f' -> target {math.degrees(target):.1f}')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.success, result.final_error = False, float(normalize_angle(target - self.yaw))
                return result

            if not self.odom_ok():
                self.stop()
                self.get_logger().error('Odometry lost during rotation')
                goal_handle.abort()
                result.success, result.final_error = False, float('nan')
                return result

            error = normalize_angle(target - self.yaw)
            # 180 deg case: +pi and -pi are the same angle, so keep the requested direction
            if abs(error) > math.pi - 0.1:
                error = math.copysign(abs(error), angle)

            if abs(error) < self.yaw_tol:
                self.stop()
                goal_handle.succeed()
                result.success, result.final_error = True, float(error)
                self.get_logger().info(f'Rotation done, error {math.degrees(error):.2f} deg')
                return result

            omega = max(-self.max_omega, min(self.max_omega, self.k_yaw * error))
            if abs(omega) < self.min_omega:
                omega = math.copysign(self.min_omega, error)

            cmd = Twist()
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)
            feedback.remaining = float(error)
            goal_handle.publish_feedback(feedback)
            time.sleep(self.period)

        self.stop()
        goal_handle.abort()
        result.success, result.final_error = False, float('nan')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Rotation()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()