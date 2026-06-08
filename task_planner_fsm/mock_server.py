import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import time

class MockServer(Node):
    def __init__(self):
        super().__init__('mock_server')
        self.srv = self.create_service(SetBool, '/start_mapping', self.handle_request_mapping)
        self.get_logger().info("Server /start_mapping ready.")
        self.get_logger().info("\033[1;32mUse: ros2 service call /start_mapping example_interfaces/srv/SetBool \"{data: true}\"\033[0m")

        self.srv = self.create_service(SetBool, '/start_geometry_reconstruction', self.handle_request_geometry_reconstruction)
        self.get_logger().info("Server /start_geometry_reconstruction ready.")
        self.get_logger().info("\033[1;32mUse: ros2 service call /start_geometry_reconstruction example_interfaces/srv/SetBool \"{data: true}\"\033[0m")

        self.srv = self.create_service(SetBool, '/compute_areas_of_interest', self.handle_request_areas_of_interest)
        self.get_logger().info("Server /compute_areas_of_interest ready.")
        self.get_logger().info("\033[1;32mUse: ros2 service call /compute_areas_of_interest example_interfaces/srv/SetBool \"{data: true}\"\033[0m")

        self.srv = self.create_service(SetBool, '/object_id_sim', self.handle_request_object_id_sim)
        self.get_logger().info("Server /object_id_sim ready.")
        self.get_logger().info("\033[1;32mUse: ros2 service call /object_id_sim example_interfaces/srv/SetBool \"{data: true}\"\033[0m")

        self.srv = self.create_service(SetBool, '/wall_lines_computation', self.handle_request_wall_lines_computation)
        self.get_logger().info("Server /wall_lines_computation ready.")
        self.get_logger().info("\033[1;32mUse: ros2 service call /wall_lines_computation example_interfaces/srv/SetBool \"{data: true}\"\033[0m")

        self.srv = self.create_service(SetBool, '/sensor_data_processing', self.handle_request_sensor_data_processing)
        self.get_logger().info("Server /sensor_data_processing ready.")
        self.get_logger().info("\033[1;32mUse: ros2 service call /sensor_data_processing example_interfaces/srv/SetBool \"{data: true}\"\033[0m")

        self.srv = self.create_service(SetBool, '/send_data_to_pokeye', self.handle_request_send_data_to_pokeye)
        self.get_logger().info("Server /send_data_to_pokeye ready.")
        self.get_logger().info("\033[1;32mUse: ros2 service call /send_data_to_pokeye example_interfaces/srv/SetBool \"{data: true}\"\033[0m")

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
    
    def handle_request_object_id_sim(self, request, response):
        self.get_logger().info("Received request to start object ID.")
        delay = 1   # seconds
        self.get_logger().info(f"Waiting {delay} seconds.")
        time.sleep(delay)
        response.success = True
        response.message = "Object ID completed succesfully."
        return response

    def handle_request_wall_lines_computation(self, request, response):
        self.get_logger().info("Received request to start wall lines computation.")
        delay = 1   # seconds
        self.get_logger().info(f"Waiting {delay} seconds.")
        time.sleep(delay)
        response.success = True
        response.message = "Wall lines computed succesfully."
        return response
    
    def handle_request_sensor_data_processing(self, request, response):
        self.get_logger().info("Received request to start sensor data processing.")
        delay = 1   # seconds
        self.get_logger().info(f"Waiting {delay} seconds.")
        time.sleep(delay)
        response.success = True
        response.message = "Sensor data processed succesfully."
        return response

    def handle_request_send_data_to_pokeye(self, request, response):
        self.get_logger().info("Received request to send data to Pokeye.")
        delay = 1   # seconds
        self.get_logger().info(f"Waiting {delay} seconds.")
        time.sleep(delay)
        response.success = True
        response.message = "Data sent to Pokeye succesfully."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
