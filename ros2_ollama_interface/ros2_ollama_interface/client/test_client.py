# File: test_client.py

import rclpy
from rclpy.node import Node
from ros2_ollama_msgs.srv import RunPrompt
import yaml
import os

class LLMClientNode(Node):
    def __init__(self):
        super().__init__('llm_client')

        # Load config with defaults
        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        config = {
            "namespace": "",
            "service_name": "run_prompt"
        }

        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as file:
                    user_config = yaml.safe_load(file)
                    if user_config:
                        for key, value in user_config.items():
                            if value not in [None, ""]:
                                config[key] = value
                            else:
                                self.get_logger().warn(f"Config key '{key}' is empty or None. Using default value: {config[key]}")
            except Exception as e:
                raise RuntimeError(f"Failed to load config.yaml: {e}")

        # Construct full service name using standard namespace prefix
        ns = config.get("namespace", "").strip("/")
        base = config.get("service_name", "run_prompt")
        prefix_ns = "ollama_interface"
        if ns:
            self.service_name = f"/{prefix_ns}/{ns}/{base}"
        else:
            self.service_name = f"/{prefix_ns}/{base}"

        # Create the service client
        self.client = self.create_client(RunPrompt, self.service_name)
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'Waiting for LLM service at {self.service_name}...')

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

if __name__ == '__main__':
    main()

