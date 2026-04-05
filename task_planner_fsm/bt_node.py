import argparse
import sys

import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from task_planner_fsm.bt.tree import build_initial_bt_tree
from task_planner_fsm.context import FsmContext


class RobotBTNode(Node):
    def __init__(self, sim: bool = False):
        super().__init__("robot_bt_node")
        self.ctx: FsmContext = {
            "node": self,
            "start": False,
            "map_ready": False,
            "error_triggered": False,
            "sim": bool(sim),
            "scan_phase": 1,
        }

        self.current_pub = self.create_publisher(String, "/fsm/current_state", 10)
        self.transition_pub = self.create_publisher(String, "/fsm/transition", 10)
        self.ctx["publish_fsm_current"] = self.publish_fsm_current
        self.ctx["publish_fsm_transition"] = self.publish_fsm_transition

        self.tree = build_initial_bt_tree(self.ctx)
        self.tree.setup(timeout=15.0)
        self.create_subscription(Bool, "/start_flag", self.start_callback, 10)
        self.timer = self.create_timer(1.0, self.tick)

    def publish_fsm_current(self, state_name: str):
        msg = String()
        msg.data = state_name
        self.current_pub.publish(msg)

    def publish_fsm_transition(self, from_state: str, to_state: str, reason: str = ""):
        msg = String()
        msg.data = f'{from_state}->{to_state}:{reason}'
        self.transition_pub.publish(msg)

    def start_callback(self, msg: Bool):
        self.ctx["start"] = msg.data

    def tick(self):
        self.tree.tick()
        current = getattr(self.tree.root.tip(), "name", None)
        if current:
            self.publish_fsm_current(current)
        if self.tree.root.status in (
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        ):
            self.get_logger().info(f"[BT] Tree finished with status: {self.tree.root.status}")


def main(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--sim", type=str, default="false", choices=["true", "false"])
    argv = args if args is not None else sys.argv[1:]
    parsed_args, remaining_args = parser.parse_known_args(argv)
    sim = parsed_args.sim.lower() == "true"

    rclpy.init(args=remaining_args)
    node = RobotBTNode(sim=sim)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
