"""Node 2: image-based visual servoing (IBVS) as a ROS 2 action server.

Goal: go ahead until a marker whose id is NOT in ignore_ids is visible,
then align with it and approach it until its apparent side s reaches sd.
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo

from erl1_interfaces.action import ApproachMarker
from erl1_interfaces.msg import MarkerDetectionArray


def clamp(value, limit):
    """Limit value to the range [-limit, +limit]."""
    return max(-limit, min(limit, value))


class VisualServoing(Node):

    def __init__(self):
        super().__init__('visual_servoing')

        # --- Parameters (all overridable from a yaml / launch file) ---
        params = {
            'detections_topic': '/aruco/detections',
            'camera_info_topic': '/camera/camera_info',
            'cmd_vel_topic': '/cmd_vel',
            'desired_side': 166.0,       # sd in simulation [px]
            'reference_fx': 381.36,      # fx of the simulated camera [px]
            'k_omega': 0.003,            # align gain   [rad/s per px]
            'k_v': 0.004,                # approach gain [m/s per px]
            'max_omega': 0.6,            # [rad/s]
            'max_v': 0.3,                # [m/s]
            'search_speed': 0.2,         # [m/s]
            'align_tolerance': 30.0,     # |u - cx| below this -> start approaching [px]
            'side_tolerance': 4.0,       # |sd - s| below this -> goal reached [px]
            'detection_timeout': 0.5,    # detections older than this are ignored [s]
            'lost_timeout': 2.0,         # how long a lost marker stays locked [s]
            'rate': 20.0,                # control loop frequency [Hz]
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in params}
        self.desired_side_sim = p['desired_side']
        self.reference_fx = p['reference_fx']
        self.k_omega, self.k_v = p['k_omega'], p['k_v']
        self.max_omega, self.max_v = p['max_omega'], p['max_v']
        self.search_speed = p['search_speed']
        self.align_tol, self.side_tol = p['align_tolerance'], p['side_tolerance']
        self.detection_timeout, self.lost_timeout = p['detection_timeout'], p['lost_timeout']
        self.period = 1.0 / p['rate']

        # --- Latest data from the subscriptions ---
        self.detections = None
        self.detections_time = 0.0
        self.fx = None   # from camera_info
        self.cx = None   # principal point (image center) from camera_info

        # Reentrant group: subscriptions keep running while execute() loops
        group = ReentrantCallbackGroup()
        self.create_subscription(MarkerDetectionArray, p['detections_topic'],
                                 self.on_detections, 10, callback_group=group)
        self.create_subscription(CameraInfo, p['camera_info_topic'],
                                 self.on_camera_info, 10, callback_group=group)
        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)

        self.server = ActionServer(
            self, ApproachMarker, 'approach_marker',
            execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('Visual servoing ready (action: /approach_marker)')

    # ---------------- Subscriptions ----------------
    def on_detections(self, msg):
        self.detections = msg
        self.detections_time = time.monotonic()

    def on_camera_info(self, msg):
        if self.fx is None:
            self.get_logger().info(f'Camera fx = {msg.k[0]:.1f}, cx = {msg.k[2]:.1f}')
        self.fx = msg.k[0]
        self.cx = msg.k[2]

    # ---------------- Helpers ----------------
    def desired_side(self):
        """sd scaled with the focal length: same stop distance on any camera."""
        if self.fx is None:
            return self.desired_side_sim
        return self.desired_side_sim * self.fx / self.reference_fx

    def image_center(self):
        if self.cx is not None:
            return self.cx
        return self.detections.image_width / 2.0

    def pick_marker(self, target_id, ignore_ids):
        """Return the marker to servo on, or None if there is none."""
        if self.detections is None:
            return None
        if time.monotonic() - self.detections_time > self.detection_timeout:
            return None
        if target_id is not None:
            candidates = [m for m in self.detections.markers if m.id == target_id]
        else:
            candidates = [m for m in self.detections.markers if m.id not in ignore_ids]
        # The biggest marker is the closest one
        return max(candidates, key=lambda m: m.side, default=None)

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------- Action ----------------
    def execute(self, goal_handle):
        ignore_ids = set(goal_handle.request.ignore_ids)
        self.get_logger().info(f'Approach goal received, ignoring ids {sorted(ignore_ids)}')
        feedback = ApproachMarker.Feedback()
        result = ApproachMarker.Result()
        target_id = None          # the marker we are locked on
        last_seen = time.monotonic()
        last_v = 0.0              # last forward speed sent while the marker was visible

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.success, result.marker_id = False, -1
                return result

            marker = self.pick_marker(target_id, ignore_ids)
            cmd = Twist()

            if marker is not None:
                target_id = marker.id
                last_seen = time.monotonic()
                error_u = marker.u - self.image_center()     # u - cx
                error_s = self.desired_side() - marker.side  # sd - s

                cmd.angular.z = clamp(-self.k_omega * error_u, self.max_omega)
                if abs(error_u) > self.align_tol:
                    feedback.phase = 'align'
                else:
                    feedback.phase = 'approach'
                    cmd.linear.x = clamp(self.k_v * error_s, self.max_v)
                last_v = cmd.linear.x

                feedback.marker_id = marker.id
                feedback.side_error = float(error_s)

                if abs(error_s) < self.side_tol and abs(error_u) <= self.align_tol:
                    self.stop()
                    goal_handle.succeed()
                    result.success, result.marker_id = True, marker.id
                    self.get_logger().info(
                        f'Reached marker {marker.id} (s = {marker.side:.1f} px)')
                    return result

            elif target_id is not None and time.monotonic() - last_seen < self.lost_timeout:
                # briefly lost (typically a far, tiny marker that flickers):
                # keep going straight, never backwards, never faster than the search speed
                feedback.phase = 'lost'
                cmd.linear.x = min(max(last_v, 0.0), self.search_speed)

            else:
                target_id = None
                feedback.phase = 'search'
                feedback.marker_id = -1
                cmd.linear.x = self.search_speed

            self.cmd_pub.publish(cmd)
            goal_handle.publish_feedback(feedback)
            time.sleep(self.period)

        self.stop()
        goal_handle.abort()
        result.success, result.marker_id = False, -1
        return result


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoing()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()      # never leave the robot moving
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()