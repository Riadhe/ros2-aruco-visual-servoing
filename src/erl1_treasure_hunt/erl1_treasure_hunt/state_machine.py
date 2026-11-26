"""Node 4: treasure hunt state machine.

This node is the "brain" of the hunt. It never drives the robot and never looks
at images: it calls the two action servers in order and decides what comes next.

    SEARCH -> SERVO -> REACHED -> EXECUTE -> SEARCH -> ...
                          |
                          +--> DONE (when the final marker is reached)

Interfaces
----------
Action clients:
    /approach_marker  (erl1_interfaces/action/ApproachMarker)  -> node 2, visual servoing
    /rotate           (erl1_interfaces/action/Rotate)          -> node 3, rotation
Publishers:
    reached_topic (std_msgs/Int32)   id of the marker just reached (node 1 draws it)
    status_topic  (std_msgs/String)  every state change, including "Treasure found"

Parameters
----------
instruction_ids, instruction_angles : the hunt rules (marker id -> relative angle, rad,
                                      positive = left / counter-clockwise)
final_marker_id                     : the marker that ends the hunt
start_delay                         : wait before starting, so every node is ready [s]
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32, String

from erl1_interfaces.action import ApproachMarker, Rotate


class TreasureHunt(Node):
    """Action client that runs the treasure hunt from start to finish."""

    def __init__(self):
        super().__init__('treasure_hunt')

        # --- Parameters: the hunt rules live here, not in the code ---
        # Marker 1: turn right 90 deg, marker 2: rotate 180 deg, marker 3: turn left 90 deg.
        self.declare_parameter('instruction_ids', [1, 2, 3])
        self.declare_parameter('instruction_angles', [-1.5708, 3.14159, 1.5708])
        self.declare_parameter('final_marker_id', 4)
        self.declare_parameter('reached_topic', '/hunt/reached')
        self.declare_parameter('status_topic', '/hunt/status')
        self.declare_parameter('start_delay', 3.0)   # let the other nodes start [s]

        # Build the instruction table {id: angle}. The two lists must match one to one.
        ids = self.get_parameter('instruction_ids').value
        angles = self.get_parameter('instruction_angles').value
        if len(ids) != len(angles):
            raise ValueError('instruction_ids and instruction_angles must have the same length')
        self.instructions = dict(zip(ids, angles))
        self.final_id = self.get_parameter('final_marker_id').value
        self.start_delay = self.get_parameter('start_delay').value

        # --- Publishers ---
        # "Marker X reached": node 1 then publishes /marker_image with the circle and the id
        self.reached_pub = self.create_publisher(
            Int32, self.get_parameter('reached_topic').value, 10)
        # Human-readable state, useful to follow the hunt and for the video
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        # --- Action clients: nodes 2 and 3 do the actual driving ---
        self.approach_client = ActionClient(self, ApproachMarker, 'approach_marker')
        self.rotate_client = ActionClient(self, Rotate, 'rotate')
        self.state = None   # current state name, to log only real changes

    # ---------------- Helpers ----------------
    def set_state(self, state, detail=''):
        """Log and publish a state change (repeated identical states are skipped)."""
        if state == self.state and not detail:
            return
        self.state = state
        text = f'{state} {detail}'.strip()
        self.get_logger().info(f'[STATE] {text}')
        self.status_pub.publish(String(data=text))

    def on_approach_feedback(self, feedback_msg):
        """Feedback from node 2: its phase tells SEARCH apart from SERVO.

        Node 2 reports 'search', 'align', 'approach' or 'lost'. Everything except
        'search' means a marker is being tracked, i.e. the SERVO state.
        """
        phase = feedback_msg.feedback.phase
        self.set_state('SEARCH' if phase == 'search' else 'SERVO')

    def call_action(self, client, goal, feedback_callback=None):
        """Send a goal and block until its result arrives. Returns the result or None.

        In ROS 2, sending a goal returns a *future* (a promise of an answer).
        spin_until_future_complete() processes ROS events until the future is ready.
        It is done twice: once for "goal accepted?", once for the final result.
        This keeps the hunt logic sequential and easy to read.
        """
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
        """Run the whole hunt. Only one action runs at a time, so only one node drives."""
        # Do not start before both servers exist (start order of the nodes does not matter)
        self.get_logger().info('Waiting for the action servers...')
        self.approach_client.wait_for_server()
        self.rotate_client.wait_for_server()
        time.sleep(self.start_delay)

        used_ids = []   # markers already executed: sent to node 2 as ignore_ids
        while rclpy.ok():
            # SEARCH + SERVO (node 2): find and approach the next marker not used yet
            self.set_state('SEARCH')
            result = self.call_action(
                self.approach_client,
                ApproachMarker.Goal(ignore_ids=used_ids),
                feedback_callback=self.on_approach_feedback)
            if result is None or not result.success:
                self.set_state('ERROR', 'approach failed')
                return

            # REACHED: remember the marker (each marker is executed only once)
            marker_id = result.marker_id
            used_ids.append(marker_id)
            self.set_state('REACHED', f'marker {marker_id}')
            self.reached_pub.publish(Int32(data=marker_id))   # node 1 -> /marker_image

            # DONE: the final marker ends the hunt
            if marker_id == self.final_id:
                self.set_state('DONE', f'Treasure found at marker {marker_id}!')
                return

            # A marker with no rule is simply ignored, and the search goes on
            if marker_id not in self.instructions:
                self.get_logger().warn(f'No instruction for marker {marker_id}, skipping it')
                continue

            # EXECUTE (node 3): rotate by the angle of this marker, then search again
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
        node.run()   # no spin() here: run() spins itself while waiting for each action
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()