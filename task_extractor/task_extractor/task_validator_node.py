import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from task_extractor.task_extractor import TaskExtractor
from task_extractor.task_validator import TaskValidator
from task_extractor.task_types import TaskInformation
from task_msgs.msg import Task  # Import the custom Task message

class TaskValidatorNode(Node):
    def __init__(self):
        super().__init__('task_validator_node')
        self.subscription_active = False
        self.task_extractor = TaskExtractor()
        self.task_validator = TaskValidator()
        
        # Publisher for validated tasks
        self.task_publisher = self.create_publisher(Task, 'validated_tasks', 10)

        # Timer to manage subscription state
        self.create_timer(0.1, self.manage_subscription)

    def enable_subscription(self):
        if not self.subscription_active:
            self.subscription = self.create_subscription(
                String,
                'transcription_topic',
                self.listener_callback,
                10)
            self.subscription_active = True

    def disable_subscription(self):
        if self.subscription_active:
            self.destroy_subscription(self.subscription)
            self.subscription_active = False

    def listener_callback(self, msg):
        self.disable_subscription()  # Prevent additional callbacks
        user_input = msg.data
        self.process_input(user_input)

    def process_input(self, user_input):
        tasks = self.task_extractor.extract_tasks(user_input)
        
        for i, task in enumerate(tasks):
            is_valid = self.task_validator.validate_task(task, i)
            if is_valid:
                self.get_logger().info(f"Task {i + 1} is valid: {task}")
                # Publish the validated task
                self.publish_validated_task(task)
            else:
                questions = self.task_validator.get_missing_info_questions(i)
                self.get_logger().info(f"Task {i + 1} is missing information: {questions}")
                
        self.enable_subscription()  # Re-enable subscription after processing

    def publish_validated_task(self, task: TaskInformation):
        # Create a Task message and populate it with validated task information
        task_msg = Task()
        task_msg.object_of_interest = task.object_of_interest or []
        task_msg.task_type = task.task_type.value if task.task_type else ''
        task_msg.source_location = task.source_location or []
        task_msg.target_location = task.target_location or []
        task_msg.context = task.context or ''

        # Publish the message
        self.task_publisher.publish(task_msg)
        self.get_logger().info(f"Published validated task: {task_msg}")

    def manage_subscription(self):
        if not self.subscription_active:
            self.enable_subscription()

def main(args=None):
    rclpy.init(args=args)
    node = TaskValidatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
