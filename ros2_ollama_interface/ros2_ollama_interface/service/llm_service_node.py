# File: llm_service_node.py

import rclpy
from rclpy.node import Node
from langchain_ollama import OllamaLLM
from langchain.prompts import PromptTemplate
from ros2_ollama_msgs.srv import RunPrompt
import yaml
import os
import threading

class LLMServiceNode(Node):
    def __init__(self):
        super().__init__('llm_service_node')

        # Load config with defaults
        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        config = {
            "model": "gemma2",
            "temperature": 0.1,
            "device": "cuda:0",
            "throttling_mode": "mutex",
            "service_name": "run_prompt"
        }

        # Override defaults from config file if it exists
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

        self.model_name = config["model"]
        self.temperature = config["temperature"]
        self.device = config["device"]
        self.throttling_mode = config["throttling_mode"]
        base_service_name = config.get("service_name", "run_prompt")

        # Optional namespace override from config, always prefixed with 'ollama_interface'
        user_ns = config.get("namespace", "").strip("/")
        prefix_ns = "ollama_interface"
        if user_ns:
            self.service_namespace = f"{prefix_ns}/{user_ns}"
        else:
            self.service_namespace = prefix_ns

        # Determine full service name or use base if no namespace is provided
        self.service_name = f"/{self.service_namespace}/{base_service_name}"

        # Validate throttling mode
        if self.throttling_mode not in ["mutex", "reject"]:
            raise ValueError(f"Invalid throttling_mode: '{self.throttling_mode}'. Must be one of: mutex, reject")

        # Initialize Ollama LLM
        self.llm = OllamaLLM(model=self.model_name, temperature=self.temperature)

        # Setup for throttling modes
        self.lock = threading.Lock()
        self.busy = False

        # Create the service
        self.srv = self.create_service(RunPrompt, self.service_name, self.run_prompt_callback)
        self.get_logger().info(f"LLM Service Node is ready with model '{self.model_name}' on device '{self.device}'.")
        self.get_logger().info(f"Throttling mode: {self.throttling_mode}. Service available at: {self.service_name}")

    def run_prompt_callback(self, request, response):
        self.get_logger().info("Service callback invoked.")

        # Handle request based on throttling mode
        if self.throttling_mode == "mutex":
            with self.lock:
                return self.handle_request(request, response)

        elif self.throttling_mode == "reject":
            if self.busy:
                response.result = "Service busy. Try again later."
                return response
            else:
                self.busy = True
                try:
                    return self.handle_request(request, response)
                finally:
                    self.busy = False

        else:
            response.result = f"Unknown throttling mode: {self.throttling_mode}"
            return response

    def handle_request(self, request, response):
        # Handle the actual model inference and respond
        try:
            prompt = PromptTemplate(input_variables=request.keys, template=request.prompt_template)
            final_prompt = prompt.format(**dict(zip(request.keys, request.variables)))
            self.get_logger().info(f"Executing prompt: {final_prompt}")
            result = self.llm.invoke(final_prompt)
            response.result = result
            self.get_logger().info(f"Prompt result: {result}")
        except Exception as e:
            self.get_logger().error(f"Prompt execution failed: {e}")
            response.result = f"Error: {str(e)}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LLMServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

