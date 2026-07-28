import time

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

# Mock services for the pokeye FSM workflow. Each entry maps a service name to
# the success message returned. Every handler simply waits `DELAY` seconds and
# replies success=True, so the FSM's happy path can be exercised end-to-end
# without the real subsystems running.
SERVICES = {
    "/receive_nav2_map": "Nav2 map received successfully.",
    "/get_semantic_map": "Semantic map received successfully.",
    "/wait_for_data": "Data received successfully.",
    "/target_selection": "Target selected successfully.",
    "/manipulator_folding": "Manipulator folded successfully.",
    "/base_placement_computation": "Base placement computed successfully.",
    "/navigate_to_target": "Target reached successfully.",
    "/manipulator_reachability": "Manipulator reachability computed successfully.",
    # NearbyPointSelection no longer calls a service — it searches the vicinity of
    # the drill point with the gantry IK, so no mock is needed here.
    # ManipulatorUnfolding no longer calls a service — it commands the gantry
    # controller directly (like ManipulatorFolding), so no mock is needed here.
    "/drill_approach": "Drill approach completed successfully.",
    "/start_suction_drill": "Suction and drill started successfully.",
    "/drill": "Drilling completed successfully.",
    "/take_out_drill": "Drill taken out successfully.",
    "/stop_suction_drill": "Suction and drill stopped successfully.",
    "/drill_retract": "Drill retracted successfully.",
    "/sample_scanning": "Sample scanned successfully.",
    "/storing_to_database": "Data stored to database successfully.",
    # HomePosition no longer calls a service — it drives the base back to the
    # start pose through the nav2 NavigateToPose action.
}

DELAY = 1.0  # seconds each mock service waits before replying


class MockServer(Node):
    def __init__(self):
        super().__init__("mock_server")
        # Keep a reference to every service so they are not garbage collected.
        # (Node.services is a read-only property, so use a private name.)
        self._services = []
        for service_name, success_message in SERVICES.items():
            srv = self.create_service(
                SetBool,
                service_name,
                self._make_handler(service_name, success_message),
            )
            self._services.append(srv)
            self.get_logger().info(f"Server {service_name} ready.")
            self.get_logger().info(
                f'\033[1;32mUse: ros2 service call {service_name} '
                'example_interfaces/srv/SetBool "{data: true}"\033[0m'
            )

    def _make_handler(self, service_name, success_message):
        def handler(request, response):
            self.get_logger().info(f"Received request on {service_name}.")
            self.get_logger().info(f"Waiting {DELAY} seconds.")
            time.sleep(DELAY)
            response.success = True
            response.message = success_message
            return response

        return handler


def main(args=None):
    rclpy.init(args=args)
    node = MockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
