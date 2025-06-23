import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import time

class MockServer(Node):
    def __init__(self):
        super().__init__('mock_server')
        self.srv = self.create_service(SetBool, '/start_mapping', self.handle_request)
        self.get_logger().info("Servidor /start_mapping listo.")

    def handle_request(self, request, response):
        self.get_logger().info("Solicitud recibida para iniciar mapeo.")
        delay = 5   # seconds
        self.get_logger().info(f"Esperando {delay} segundos.")
        time.sleep(delay)
        response.success = True
        response.message = "Mapa generado exitosamente"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
