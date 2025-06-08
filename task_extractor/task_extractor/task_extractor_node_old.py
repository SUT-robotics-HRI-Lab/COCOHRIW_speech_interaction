import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_ollama_msgs.srv import RunPrompt
from langchain.prompts import PromptTemplate
from pydantic import TypeAdapter
import json
import re
import os
import yaml
from typing import List

from task_extractor.task_types import TaskInformation
from task_msgs.msg import Task  # Custom ROS 2 message

class TaskExtractorNode(Node):
    def __init__(self):
        super().__init__('task_extractor_node')

        # Load config
        self.config = {
            "model": "gemma2",
            "temperature": 0.1,
            "service_name": "run_prompt",
            "namespace": "ollama_interface"
        }

        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    user_config = yaml.safe_load(f)
                    for key, value in user_config.items():
                        if value not in [None, ""]:
                            self.config[key] = value
                        else:
                            self.get_logger().warn(f"Empty config key: {key}. Using default: {self.config[key]}")
            except Exception as e:
                self.get_logger().error(f"Failed to load config.yaml: {e}")

        self.service_path = f"/{self.config['namespace'].strip('/')}/{self.config['service_name']}"

        # Create LLM service client
        self.cli = self.create_client(RunPrompt, self.service_path)
        while not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(f"Waiting for {self.service_path} service...")

        # Create publisher for raw task messages
        self.task_publisher = self.create_publisher(Task, '/task_extractor/task_raw', 10)

        # Prompt template for task extraction
        self.prompt = PromptTemplate(
            input_variables=["user_input"],
            template="""
                As an intelligent assistant, break down each part of the user's request into distinct tasks. Each task should be identified as one of the following types:
                - Relocate_Object, Navigate, Inspect, Identify, Monitor, Assist, Report, Follow.

                Each task should be strictly outputted only in the following JSON format:
                {{
                    "object_of_interest": ["<object1>", "<object2>", ...],
                    "task_type": "<task from list above>",
                    "source_location": ["<source1>", "<source2>", ...],
                    "target_location": ["<target1>", "<target2>", ...],
                    "context": "<context>"
                }}

                For multiple tasks, return a list of JSON objects. Do not include any explanation or extra text.

                User Request: "{user_input}"
                Output:
            """
        )

        # Subscribe to the transcription topic
        self.subscription = self.create_subscription(
            String,
            'transcription_topic',
            self.listener_callback,
            10
        )

        self.get_logger().info(f'Task Extractor Node started with model="{self.config["model"]}"')
        self.get_logger().info(f'LLM service: {self.service_path}')

    def listener_callback(self, msg: String):
        user_input = msg.data
        self.get_logger().info(f"Received transcription: {user_input}")

        tasks = self.extract_tasks(user_input)

        for i, task in enumerate(tasks, start=1):
            self.get_logger().info(f"Extracted Task {i}:\n{task.json(indent=2)}")
            ros_msg = self.convert_to_msg(task)
            self.task_publisher.publish(ros_msg)
            self.get_logger().info(f"Published Task {i} to /task_extractor/task_raw")

    def extract_tasks(self, user_input: str) -> List[TaskInformation]:
        formatted_prompt = self.prompt.format(user_input=user_input)

        request = RunPrompt.Request()
        request.prompt_template = self.prompt.template
        request.keys = ['user_input']
        request.variables = [user_input]

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('LLM service call failed.')
            return []

        response_text = future.result().result
        json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
        if not json_match:
            self.get_logger().error("No JSON array found in LLM response.")
            return []

        try:
            cleaned_json = json_match.group(0).replace('"null"', 'null')
            task_data = json.loads(cleaned_json)
            self.get_logger().info(f"Parsed task data: {task_data}")
            return TypeAdapter(List[TaskInformation]).validate_python(task_data)
        except Exception as e:
            self.get_logger().error(f"Error parsing task list: {e}")
            return []

    def convert_to_msg(self, task: TaskInformation) -> Task:
        msg = Task()
        msg.object_of_interest = task.object_of_interest or []
        msg.task_type = task.task_type.value if task.task_type else ""
        msg.source_location = task.source_location or []
        msg.target_location = task.target_location or []
        msg.context = task.context or ""
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = TaskExtractorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
