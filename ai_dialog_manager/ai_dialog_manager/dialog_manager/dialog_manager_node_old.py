# dialog_manager_node.py

import rclpy
from rclpy.node import Node
from dialog_interfaces.msg import DialogState
from task_msgs.msg import Task, TaskArray
from std_msgs.msg import String
from task_extractor.task_types import TaskInformation
from .task_completion_handler import TaskCompletionHandler, TaskField
from .dialog_llm_interface import DialogLLMInterface
from coqui_tts_interfaces.srv import Speak
import queue

class DialogManagerNode(Node):
    def __init__(self):
        super().__init__('dialog_manager_node')

        # ROS 2 publishers and subscribers
        self.dialog_state_pub = self.create_publisher(DialogState, '/dialog_state', 10)
        self.task_sub = self.create_subscription(Task, '/task_extractor/task_raw', self.task_callback, 10)
        self.task_array_sub = self.create_subscription(TaskArray, '/task_extractor/raw_task_list', self.task_array_callback, 10)
        self.clarification_sub = self.create_subscription(String, '/transcription_classifier/clarification', self.clarification_callback, 10)
        self.dialog_state_sub = self.create_subscription(DialogState, '/dialog_state', self.dialog_state_callback, 10)

        # Coqui TTS service client
        self.tts_client = self.create_client(Speak, '/coqui_tts/synthesize')
        while not self.tts_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for TTS service '/coqui_tts/synthesize'...")

        # Task and dialog tracking
        self.task_handler = TaskCompletionHandler()
        self.task_index_counter = 0
        self.last_incomplete_index = None
        self.expecting_clarification = False
        self.current_dialog_state = "AWAITING_TASK"

        # Task queuing and flow control
        self.processing_task = False
        self.task_queue = queue.Queue()

        # LLM interface
        self.llm_interface = DialogLLMInterface(self, '/ollama_interface/run_prompt')

        # Publisher for semantically complete tasks
        self.task_complete_pub = self.create_publisher(Task, '/dialog_manager/task_complete', 10)

        self.get_logger().info("Dialog Manager Node initialized.")

    def dialog_state_callback(self, msg: DialogState):
        # Keep track of the most recent dialog state
        self.current_dialog_state = msg.current_state
        self.get_logger().info(f"🧠 Dialog state updated: {self.current_dialog_state}")

    def task_callback(self, msg: Task):
        # Only accept new tasks when in the AWAITING_TASK state
        if self.current_dialog_state != "AWAITING_TASK":
            self.get_logger().warn("🚫 Not in AWAITING_TASK state. Ignoring incoming task.")
            return

        if self.processing_task:
            self.get_logger().info("⚠️ Currently processing a task. Queuing new task.")
            self.task_queue.put(msg)
            return

        self.processing_task = True

        # Convert message to TaskInformation
        task = TaskInformation(
            object_of_interest=msg.object_of_interest,
            task_type=msg.task_type if msg.task_type else None,
            source_location=msg.source_location,
            target_location=msg.target_location,
            context=msg.context
        )

        task_index = self.task_index_counter
        self.task_index_counter += 1

        self.process_task(task, task_index)

    def clarification_callback(self, msg: String):
        # Validate context
        if not self.expecting_clarification:
            self.get_logger().warn("Received clarification but system is not expecting one.")
            return

        if self.last_incomplete_index is None:
            self.get_logger().warn("Received clarification but no task is awaiting clarification.")
            return

        # Extract user input and track missing fields
        user_input = msg.data.strip()
        self.get_logger().info(f"📥 Received clarification input: {user_input}")
        missing_fields = self.task_handler.get_missing_fields(self.last_incomplete_index)

        # Define inline callback for LLM response
        def on_extracted_fields(task_data, error):
            if error or not task_data:
                self.get_logger().error(f"❌ Failed to extract clarification info: {error}")
                return

            current = self.task_handler.get_task(self.last_incomplete_index)
            debug_lines = []

            if task_data.object_of_interest and not current.object_of_interest:
                current.object_of_interest = task_data.object_of_interest
                debug_lines.append(f"✅ Filled object_of_interest: {task_data.object_of_interest}")

            if task_data.source_location and not current.source_location:
                current.source_location = task_data.source_location
                debug_lines.append(f"✅ Filled source_location: {task_data.source_location}")

            if task_data.target_location and not current.target_location:
                current.target_location = task_data.target_location
                debug_lines.append(f"✅ Filled target_location: {task_data.target_location}")

            if task_data.task_type and not current.task_type:
                current.task_type = task_data.task_type
                debug_lines.append(f"✅ Filled task_type: {task_data.task_type}")

            if debug_lines:
                for line in debug_lines:
                    self.get_logger().info(line)
            else:
                self.get_logger().warn("⚠️ No new valid fields were found in clarification.")

            self.task_handler.update_task(self.last_incomplete_index, current)
            self.process_task(current, self.last_incomplete_index)

        # Send input to LLM for extraction of relevant task info
        self.llm_interface.request_task_field_extraction(user_input, missing_fields, on_extracted_fields)(user_input, missing_fields, on_extracted_fields)

    def process_task(self, task: TaskInformation, task_index: int):
        # Validate the task and check what’s still missing
        is_valid, clarification_sentence, missing_fields = self.task_handler.register_task(task, task_index)

        # Update dialog state
        state_msg = DialogState()
        state_msg.previous_state = self.current_dialog_state
        state_msg.current_state = "CONFIRMING_TASK" if is_valid else "AWAITING_CLARIFICATION"
        self.dialog_state_pub.publish(state_msg)

        if is_valid:
            # Publish task if complete
            self.get_logger().info("✅ Task is complete. Moving to confirmation.")
            complete_task_msg = Task()
            complete_task_msg.object_of_interest = task.object_of_interest or []
            complete_task_msg.task_type = task.task_type.value if task.task_type else ""
            complete_task_msg.source_location = task.source_location or []
            complete_task_msg.target_location = task.target_location or []
            complete_task_msg.context = task.context or ""
            self.task_complete_pub.publish(complete_task_msg)

            # Reset state
            self.processing_task = False
            self.expecting_clarification = False
            self.process_next_task_if_any()
        else:
            # Handle incomplete task
            if task_index == self.last_incomplete_index:
                self.get_logger().warn("❌ Task still incomplete after clarification.")
            else:
                self.get_logger().warn("❌ Task is incomplete. Clarification needed.")
                self.last_incomplete_index = task_index

            self.get_logger().warn(f"Clarification: {clarification_sentence}")
            self.get_logger().warn(f"Missing fields: {[field.value for field in missing_fields]}")
            self.expecting_clarification = True

            # Ask the user for clarification via TTS
            self.llm_interface.request_clarification_text(clarification_sentence, self.handle_llm_response)

    def handle_llm_response(self, response_text: str, error):
        if error:
            self.get_logger().error(f"LLM Clarification Error: {error}")
            return

        self.get_logger().info(f"🔁 Asking user: {response_text}")
        req = Speak.Request()
        req.text = response_text

        future = self.tts_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info("✅ TTS response sent."))

    def task_array_callback(self, msg: TaskArray):
        if self.processing_task or not self.task_queue.empty():
            self.get_logger().warn("🚫 Task queue is not empty or a task is being processed. Ignoring incoming task array.")
            return
        for task in msg.tasks:
            self.task_callback(task)

    def process_next_task_if_any(self):
        if not self.task_queue.empty():
            next_msg = self.task_queue.get()
            self.get_logger().info("➡️ Processing next task from queue.")
            self.task_callback(next_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DialogManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
