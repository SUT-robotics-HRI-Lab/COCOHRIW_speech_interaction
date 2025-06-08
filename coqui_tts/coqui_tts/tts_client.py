import rclpy
from rclpy.node import Node
from coqui_tts_interfaces.srv import Speak

class TTSClientNode(Node):
    def __init__(self):
        super().__init__('tts_client_node')
        self.cli = self.create_client(Speak, '/coqui_tts/synthesize')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = Speak.Request()

    def send_request(self, text):
        self.req.text = text
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = TTSClientNode()
    text = input("Enter text to synthesize: ")
    response = node.send_request(text)
    print("Success:", response.success)
    rclpy.shutdown()
