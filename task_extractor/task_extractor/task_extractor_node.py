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
from typing import List, Optional

from task_extractor.task_types import TaskInformation, TaskType  # Import the TaskInformation and TaskType classes
from task_msgs.msg import Task
from task_msgs.msg import TaskArray  # import the new message

class TaskExtractorNode(Node):
    def __init__(self):
        super().__init__('task_extractor_node')

        # Initialize the node and load configuration parameters from YAML or defaults
        self.config = {
            "temperature": 0.1,
            "service_name": "run_prompt",
            "namespace": "ollama_interface",
            "transcription_topic": "/transcription_classifier/new_task"
        }

        # Load configuration from a YAML file if it exists
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
        else:
            self.get_logger().warn("No config.yaml found. Using default configuration.")

        # Build the full path to the LLM service
        self.service_path = f"/{self.config['namespace'].strip('/')}/{self.config['service_name']}"
        self.cli = self.create_client(RunPrompt, self.service_path)

        # Wait for the LLM service to become available
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Waiting for LLM service at {self.service_path}...")

        # Create a publisher to broadcast raw extracted tasks
        self.task_publisher = self.create_publisher(Task, '/task_extractor/task_raw', 10)

        # Create a publisher for task arrays
        self.task_array_publisher = self.create_publisher(TaskArray, '/task_extractor/raw_task_list', 10)

        # Define the prompt template for task extraction using LangChain
        task_type_options = ', '.join([t.value for t in TaskType])
        self.prompt = PromptTemplate(
            input_variables=["user_input", "task_type_options"],
            template="""
                As an intelligent assistant, break down each part of the user's request into distinct tasks. Each task should be identified as one of the following types:
                - "{task_type_options}".

                Be precise. Fit the user's request as accurately as possible into one of the task types. If the instruction does not match any of these types, still generate a response: assign it the task_type 'Unknown' and leave all other fields empty or null.

                Example:
                User Request: "Lets do something today" or "Lets do some tasks"
                Output:
                {{
                    "object_of_interest": null,
                    "task_type": "Unknown",
                    "source_location": null,
                    "target_location": null,
                    "context": "Lets do something today"
                }}


                Distinguish clearly between:
                - Relocate_Object: user specifies both a source and a target, like: "move the book from the table to the shelf" and uses worlds like "move", "take", "put", "place", "transfer".
                - Pick: user instructs to grab something without saying where to put it, like: "pick up the book" or "grab the cup", and uses words like "pick", "pick up", "grab", "take".
                - Place: user instructs to place something with a target or leaves destination vague, like: "put it on the table", "place it in the box", "put it down", "release the object", and uses words like "put", "place", "drop", "release".
                
                Each task should be strictly outputted only in the following JSON format:
                {{
                    "object_of_interest": ["<object1>", "<object2>", ...],
                    "task_type": "<task from list above>",
                    "source_location": ["<source1>", "<source2>", ...],
                    "target_location": ["<target1>", "<target2>", ...],
                    "context": "<context>"
                }}

                Example 1:
                User Request: "Go to the pantry, pick up a bottle of water and a snack, take them to the gym, and place them on the shelf and bench."
                Output:
                [
                    {{
                        "object_of_interest": null,
                        "task_type": "Navigate",
                        "source_location": null,
                        "target_location": ["pantry"],
                        "context": "Go to the pantry"
                    }},
                    {{
                        "object_of_interest": ["bottle of water", "snack"],
                        "task_type": "Relocate_Object",
                        "source_location": ["pantry"],
                        "target_location": ["gym"],
                        "context": "pick up a bottle of water and a snack and take them to the gym"
                    }},
                    {{
                        "object_of_interest": ["bottle of water", "snack"],
                        "task_type": "Relocate_Object",
                        "source_location": ["gym"],
                        "target_location": ["shelf", "bench"],
                        "context": "place them on the shelf and bench"
                    }}
                ]

                Example 2:
                User Request: "Go to the office and garage, inspect the printer and toolbox, and report back to me."
                Output:
                [
                    {{
                        "object_of_interest": null,
                        "task_type": "Navigate",
                        "source_location": null,
                        "target_location": ["office", "garage"],
                        "context": "Go to the office and garage"
                    }},
                    {{
                        "object_of_interest": ["printer", "toolbox"],
                        "task_type": "Inspect",
                        "source_location": ["office", "garage"],
                        "target_location": null,
                        "context": "inspect the printer and toolbox"
                    }},
                    {{
                        "object_of_interest": ["printer", "toolbox"],
                        "task_type": "Report",
                        "source_location": ["office", "garage"],
                        "target_location": ["me"],
                        "context": "report back to me"
                    }}
                ]

                For multiple tasks, return a list of JSON objects, the order of tasks is very important and must be coherent with the request. Do not include any explanation or extra text.
                Never include explanations, thinking, headers, or additional text outside the JSON block
                User Request: "{user_input}"
                Output:
            """
        )

        # Initialize internal state variables
        self.future = None
        self.waiting_for_response = False
        self.subscription: Optional[rclpy.subscription.Subscription] = None
        self.enable_subscription()

    def enable_subscription(self):
        """
        Enable the subscription to the transcription topic.
        This allows the node to receive user input messages for processing.
        """
        if self.subscription is None:
            self.subscription = self.create_subscription(
                String,
                self.config["transcription_topic"],  # Use parameter from config
                self.transcription_callback,
                10
            )
            self.get_logger().info(f"Subscribed to '{self.config['transcription_topic']}'")

    def disable_subscription(self):
        """
        Disable the subscription to the transcription topic.
        This prevents the node from receiving new user input messages while processing.
        """
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.get_logger().info("Unsubscribed from 'transcription_classifier/new_task'")

    def transcription_callback(self, msg: String):
        """
        Callback function for handling incoming transcription messages.
        If the node is waiting for a response from the LLM, new messages are ignored.
        Otherwise, the user input is sent to the LLM for processing.
        """
        if self.waiting_for_response:
            self.get_logger().warn("Still waiting for LLM response. Ignoring new transcription input.")
            return

        user_input = msg.data
        self.get_logger().info(f"Received transcription: {user_input}")
        self.send_prompt(user_input)

    def send_prompt(self, user_input: str):
        """
        Format the user input into a prompt and send it to the LLM service.
        Disables the subscription while waiting for the response.
        """
        task_type_options = ', '.join([t.value for t in TaskType])
        formatted_prompt = self.prompt.format(
            user_input=user_input,
            task_type_options=task_type_options
        )

        request = RunPrompt.Request()
        request.prompt_template = self.prompt.template
        request.keys = ['user_input', 'task_type_options']
        request.variables = [user_input, task_type_options]

        self.future = self.cli.call_async(request)
        self.waiting_for_response = True
        self.disable_subscription()

    def check_response(self):
        """
        Check if the LLM service has returned a response.
        If a response is available, process it to extract tasks and publish them.
        Re-enable the subscription after processing the response.
        """
        if self.future and self.future.done():
            try:
                response_text = self.future.result().result
                self.get_logger().info(f"LLM response: {response_text}")

                # Extract JSON data from the response
                start = response_text.find("[")
                end = response_text.rfind("]") + 1
                json_chunk = response_text[start:end]
                cleaned_json = json_chunk.replace('"null"', 'null')
                task_data = json.loads(cleaned_json)
                tasks = TypeAdapter(List[TaskInformation]).validate_python(task_data)

                # # Publish each extracted task individually
                # for i, task in enumerate(tasks, start=1):
                #     self.get_logger().info("Extracted Task {}:\n{}".format(i, task.model_dump_json(indent=2)))
                #     msg = self.convert_to_msg(task)
                #     self.task_publisher.publish(msg)
                #     self.get_logger().info(f"Published Task {i} to /task_extractor/task_raw")

                # Publish the entire list as a TaskArray
                task_array_msg = TaskArray()
                task_array_msg.tasks = [self.convert_to_msg(task) for task in tasks]
                self.task_array_publisher.publish(task_array_msg)
                self.get_logger().info(f"Published {len(tasks)} tasks as TaskArray to /task_extractor/raw_task_list")

            except Exception as e:
                import traceback
                self.get_logger().error(f"Failed to process LLM result: {e}")
                self.get_logger().error(traceback.format_exc())
            finally:
                self.future = None
                self.waiting_for_response = False
                self.enable_subscription()

    def convert_to_msg(self, task: TaskInformation) -> Task:
        """
        Convert a TaskInformation object into a ROS2 Task message.
        This prepares the task for publishing to the task_raw topic.
        """
        msg = Task()
        msg.object_of_interest = task.object_of_interest or []
        msg.task_type = task.task_type.value if task.task_type else ""
        msg.source_location = task.source_location or []
        msg.target_location = task.target_location or []
        msg.context = task.context or ""
        return msg

def main(args=None):
    """
    Main function to initialize the ROS2 node and start spinning.
    Continuously checks for responses from the LLM service.
    """
    rclpy.init(args=args)
    node = TaskExtractorNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_response()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
