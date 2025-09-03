import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import time

class MockServer(Node):
    def __init__(self):
        super().__init__('mock_server')
        self.srv = self.create_service(SetBool, '/start_mapping', self.handle_request_mapping)
        self.get_logger().info("Server /start_mapping ready.")

        self.srv = self.create_service(SetBool, '/start_geometry_reconstruction', self.handle_request_geometry_reconstruction)
        self.get_logger().info("Server /start_geometry_reconstruction ready.")

        self.srv = self.create_service(SetBool, '/compute_areas_of_interest', self.handle_request_areas_of_interest)
        self.get_logger().info("Server /compute_areas_of_interest ready.")

    def handle_request_mapping(self, request, response):
        self.get_logger().info("Received request to start mapping.")
        delay = 1   # seconds
        self.get_logger().info(f"Waiting {delay} seconds.")
        time.sleep(delay)
        response.success = True
        response.message = "Map generated succesfully."
        return response
    
    def handle_request_geometry_reconstruction(self, request, response):
        self.get_logger().info("Received request to start geometry reconstruction.")
        delay = 1   # seconds
        self.get_logger().info(f"Waiting {delay} seconds.")
        time.sleep(delay)
        response.success = True
        response.message = "Geometry reconstructed succesfully."
        return response
    
    def handle_request_areas_of_interest(self, request, response):
        self.get_logger().info("Received request to start areas of interest computation.")
        delay = 1   # seconds
        self.get_logger().info(f"Waiting {delay} seconds.")
        time.sleep(delay)
        response.success = True
        response.message = "Areas of interest computed succesfully."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
