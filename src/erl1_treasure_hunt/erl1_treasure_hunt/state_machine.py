"""Node 4: treasure hunt state machine (action client of nodes 2 and 3).

SEARCH -> SERVO -> EXECUTE -> SEARCH ...  and  SERVO -> DONE when the final marker is reached.
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32, String

from erl1_interfaces.action import ApproachMarker, Rotate


class TreasureHunt(Node):

    def __init__(self):
        super().__init__('treasure_hunt')

        # Instructions: marker id -> relative rotation [rad] (positive = left)
        self.declare_parameter('instruction_ids', [1, 2, 3])
        self.declare_parameter('instruction_angles', [-1.5708, 3.14159, 1.5708])
        self.declare_parameter('final_marker_id', 4)
        self.declare_parameter('reached_topic', '/hunt/reached')
        self.declare_parameter('status_topic', '/hunt/status')
        self.declare_parameter('start_delay', 3.0)   # let the other nodes start [s]

        ids = self.get_parameter('instruction_ids').value
        angles = self.get_parameter('instruction_angles').value
        if len(ids) != len(angles):
            raise ValueError('instruction_ids and instruction_angles must have the same length')
        self.instructions = dict(zip(ids, angles))
        self.final_id = self.get_parameter('final_marker_id').value
        self.start_delay = self.get_parameter('start_delay').value

        self.reached_pub = self.create_publisher(
            Int32, self.get_parameter('reached_topic').value, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        self.approach_client = ActionClient(self, ApproachMarker, 'approach_marker')
        self.rotate_client = ActionClient(self, Rotate, 'rotate')
        self.state = None

    # ---------------- Helpers ----------------
    def set_state(self, state, detail=''):
        """Log and publish every state change (useful for the README and the video)."""
        if state == self.state and not detail:
            return
        self.state = state
        text = f'{state} {detail}'.strip()
        self.get_logger().info(f'[STATE] {text}')
        self.status_pub.publish(String(data=text))

    def on_approach_feedback(self, feedback_msg):
        phase = feedback_msg.feedback.phase
        self.set_state('SEARCH' if phase == 'search' else 'SERVO')

    def call_action(self, client, goal, feedback_callback=None):
        """Send a goal and block until its result arrives. Returns the result or None."""
        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    # ---------------- Main logic ----------------
    def run(self):
        self.get_logger().info('Waiting for the action servers...')
        self.approach_client.wait_for_server()
        self.rotate_client.wait_for_server()
        time.sleep(self.start_delay)

        used_ids = []
        while rclpy.ok():
            # SEARCH + SERVO (node 2)
            self.set_state('SEARCH')
            result = self.call_action(
                self.approach_client,
                ApproachMarker.Goal(ignore_ids=used_ids),
                feedback_callback=self.on_approach_feedback)
            if result is None or not result.success:
                self.set_state('ERROR', 'approach failed')
                return

            marker_id = result.marker_id
            used_ids.append(marker_id)   # each marker is executed only once
            self.set_state('REACHED', f'marker {marker_id}')
            self.reached_pub.publish(Int32(data=marker_id))   # node 1 -> /marker_image

            # DONE
            if marker_id == self.final_id:
                self.set_state('DONE', f'Treasure found at marker {marker_id}!')
                return

            if marker_id not in self.instructions:
                self.get_logger().warn(f'No instruction for marker {marker_id}, skipping it')
                continue

            # EXECUTE (node 3)
            angle = self.instructions[marker_id]
            self.set_state('EXECUTE', f'rotate {angle:+.2f} rad')
            result = self.call_action(self.rotate_client, Rotate.Goal(angle=angle))
            if result is None or not result.success:
                self.set_state('ERROR', 'rotation failed')
                return


def main(args=None):
    rclpy.init(args=args)
    node = TreasureHunt()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()