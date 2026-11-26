"""Node 2: image-based visual servoing (IBVS), as an action server.

Goal: go ahead until a marker whose id is NOT in ignore_ids is visible,
then align with it and approach it until its apparent side s reaches sd.

IBVS: the error is computed directly on the image features, with no 3D
reconstruction:
    align:     omega = -k_omega * (u - cx)     u = marker center, cx = image center
    approach:  v     =  k_v     * (sd - s)     s = marker side, sd = desired side
Pinhole model: s = f * L / d, so the same stop distance on another camera needs
    sd = desired_side * fx / reference_fx      (fx read from camera_info)

Phases inside one goal (sent as feedback):
    search   -> no usable marker in view: go straight at search_speed
    align    -> marker off-center: turn (and keep driving if the marker is far)
    approach -> marker centered: turn + drive with the IBVS laws
    lost     -> marker disappeared for less than lost_timeout: keep going slowly

Interfaces
----------
Action server:
    /approach_marker (erl1_interfaces/action/ApproachMarker)
        goal:     ignore_ids                 markers already used
        feedback: phase, marker_id, side_error
        result:   success, marker_id
Subscribers:
    detections_topic  (erl1_interfaces/MarkerDetectionArray)  from node 1
    camera_info_topic (sensor_msgs/CameraInfo)                fx and cx
Publisher:
    cmd_vel_topic (geometry_msgs/Twist)   only while a goal is running
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
    """Action server that finds a new marker and reaches it with IBVS."""

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
            'far_ratio': 0.5,            # s < far_ratio * sd -> marker is far (0 = off)
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
        self.far_ratio = p['far_ratio']
        self.period = 1.0 / p['rate']

        # --- Latest data from the subscriptions ---
        self.detections = None        # last MarkerDetectionArray from node 1
        self.detections_time = 0.0   # when it arrived (monotonic clock)
        self.fx = None   # focal length from camera_info [px]
        self.cx = None   # principal point (image center) from camera_info [px]

        # Reentrant group + multi-threaded executor: the subscriptions keep running
        # while execute() loops, otherwise the robot would act on frozen data.
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
        """Store the latest detections and when they arrived."""
        self.detections = msg
        self.detections_time = time.monotonic()

    def on_camera_info(self, msg):
        """Read the intrinsics: K = [fx 0 cx; 0 fy cy; 0 0 1] stored row by row."""
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
        """cx from camera_info, or half the image width if camera_info is missing."""
        if self.cx is not None:
            return self.cx
        return self.detections.image_width / 2.0

    def is_far(self, side):
        """A marker much smaller than sd is far away (more than about 1 m)."""
        return side < self.far_ratio * self.desired_side()

    def pick_marker(self, target_id, ignore_ids):
        """Return the marker to servo on, or None if there is none.

        - Old detections (camera delay, node 1 stopped) are ignored.
        - Once a marker is chosen (target_id), we stay locked on it.
        - Otherwise we take the biggest marker not in ignore_ids: the closest one.
        """
        if self.detections is None:
            return None
        if time.monotonic() - self.detections_time > self.detection_timeout:
            return None
        if target_id is not None:
            candidates = [m for m in self.detections.markers if m.id == target_id]
        else:
            candidates = [m for m in self.detections.markers if m.id not in ignore_ids]
        return max(candidates, key=lambda m: m.side, default=None)

    def stop(self):
        """Publish a zero velocity."""
        self.cmd_pub.publish(Twist())

    # ---------------- Action ----------------
    def execute(self, goal_handle):
        """Search, align and approach one marker. Runs for the whole approach."""
        ignore_ids = set(goal_handle.request.ignore_ids)
        self.get_logger().info(f'Approach goal received, ignoring ids {sorted(ignore_ids)}')
        feedback = ApproachMarker.Feedback()
        result = ApproachMarker.Result()
        target_id = None              # the marker we are locked on
        last_seen = time.monotonic()  # when the locked marker was last detected
        last_v = 0.0                  # last forward speed while the marker was visible
        last_side = 0.0               # last apparent size of the locked marker

        while rclpy.ok():
            # Node 4 can cancel the goal at any time
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.success, result.marker_id = False, -1
                return result

            marker = self.pick_marker(target_id, ignore_ids)
            cmd = Twist()

            if marker is not None:
                # --- Marker visible: IBVS ---
                target_id = marker.id
                last_seen = time.monotonic()
                last_side = marker.side
                error_u = marker.u - self.image_center()     # u - cx
                error_s = self.desired_side() - marker.side  # sd - s

                # Align law, always active. The minus sign turns the robot toward the
                # marker: marker on the right (u > cx) -> negative omega -> turn right.
                cmd.angular.z = clamp(-self.k_omega * error_u, self.max_omega)
                if abs(error_u) > self.align_tol:
                    # Off-center: turn first so the marker does not leave the image.
                    # Far away the heading error is small, so keep driving meanwhile.
                    feedback.phase = 'align'
                    if self.is_far(marker.side):
                        cmd.linear.x = self.search_speed
                else:
                    # Centered: approach law. It slows down by itself near the target.
                    feedback.phase = 'approach'
                    cmd.linear.x = clamp(self.k_v * error_s, self.max_v)
                last_v = cmd.linear.x

                feedback.marker_id = marker.id
                feedback.side_error = float(error_s)

                # Goal reached: right size and centered
                if abs(error_s) < self.side_tol and abs(error_u) <= self.align_tol:
                    self.stop()
                    goal_handle.succeed()
                    result.success, result.marker_id = True, marker.id
                    self.get_logger().info(
                        f'Reached marker {marker.id} (s = {marker.side:.1f} px)')
                    return result

            elif target_id is not None and time.monotonic() - last_seen < self.lost_timeout:
                # --- Marker briefly lost (typically a far, tiny marker that flickers) ---
                feedback.phase = 'lost'
                if self.is_far(last_side):
                    # Still far: keep the search speed, getting closer makes it visible again
                    cmd.linear.x = self.search_speed
                else:
                    # Close: keep the last speed, never backwards, never faster than search
                    cmd.linear.x = min(max(last_v, 0.0), self.search_speed)

            else:
                # --- No marker: go straight and look for one ---
                target_id = None
                feedback.phase = 'search'
                feedback.marker_id = -1
                cmd.linear.x = self.search_speed

            self.cmd_pub.publish(cmd)
            goal_handle.publish_feedback(feedback)
            time.sleep(self.period)

        # ROS is shutting down
        self.stop()
        goal_handle.abort()
        result.success, result.marker_id = False, -1
        return result


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoing()
    # Multi-threaded: execute() blocks one thread, the subscriptions use the others
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