import rclpy
from rclpy.node import Node
from ros2_ollama_msgs.srv import RunPrompt

class LLMClientNode(Node):
    def __init__(self):
        super().__init__('llm_client')
        self.client = self.create_client(RunPrompt, 'run_prompt_gemma2')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for LLM service...')

        self.future = None
        self.send_prompt()

    def send_prompt(self):
        request = RunPrompt.Request()
        request.prompt_template = "What is the capital of {country}?"
        request.keys = ["country"]
        request.variables = ["Slovakia"]
        self.future = self.client.call_async(request)

    def run_until_response(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.future and self.future.done():
                try:
                    result = self.future.result()
                    self.get_logger().info(f"LLM response: {result.result}")
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = LLMClientNode()
    node.run_until_response()
    node.destroy_node()
    rclpy.shutdown()

