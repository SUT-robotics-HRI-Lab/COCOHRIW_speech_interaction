# File: llm_service_node.py

import rclpy
from rclpy.node import Node
from langchain_ollama import OllamaLLM
from langchain.prompts import PromptTemplate
from ros2_ollama_msgs.srv import RunPrompt
import yaml
import os
import threading
import queue

class LLMServiceNode(Node):
    def __init__(self):
        super().__init__('llm_service_node')

        # Load and validate config
        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
        except Exception as e:
            raise RuntimeError(f"Failed to load config.yaml: {e}")

        # Validate required keys
        required_keys = ["model", "temperature", "device", "throttling_mode", "service_name"]
        for key in required_keys:
            if key not in config:
                raise ValueError(f"Missing required config key: '{key}' in config.yaml")

        self.model_name = config["model"]
        self.temperature = config["temperature"]
        self.device = config["device"]
        self.throttling_mode = config["throttling_mode"]
        self.service_name = config["service_name"]

        # Validate throttling mode
        if self.throttling_mode not in ["queue", "mutex", "reject"]:
            raise ValueError(f"Invalid throttling_mode: '{self.throttling_mode}'. Must be one of: queue, mutex, reject")

        # Initialize Ollama LLM
        self.llm = OllamaLLM(model=self.model_name, temperature=self.temperature, device=self.device)

        # Setup for throttling modes
        self.lock = threading.Lock()
        self.queue = queue.Queue()
        self.busy = False

        # Start worker thread if using queue mode
        if self.throttling_mode == "queue":
            self.worker_thread = threading.Thread(target=self.process_queue)
            self.worker_thread.daemon = True
            self.worker_thread.start()

        # Create the service with configured name
        self.srv = self.create_service(RunPrompt, self.service_name, self.run_prompt_callback)
        self.get_logger().info(f"LLM Service Node is ready with model '{self.model_name}' on device '{self.device}'. Throttling mode: {self.throttling_mode}. Service name: {self.service_name}")

    def run_prompt_callback(self, request, response):
        self.get_logger().info("Service callback invoked.")

        # Handle request based on throttling mode
        if self.throttling_mode == "queue":
            self.queue.put((request, response))
            return response

        elif self.throttling_mode == "mutex":
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

    def process_queue(self):
        # Continuously process queued requests in a separate thread
        while True:
            request, response = self.queue.get()
            try:
                self.handle_request(request, response)
            except Exception as e:
                self.get_logger().error(f"Queue processing failed: {e}")
            self.queue.task_done()

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

