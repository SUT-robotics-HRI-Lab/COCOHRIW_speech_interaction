#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from task_extractor.msg import Task
from task_types import TaskInformation
from task_validator import TaskValidator
from task_extractor import TaskExtractor

class TaskValidatorNode:
    def __init__(self):
        # Initialize the ROS 1 node
        rospy.init_node("task_validator_node", anonymous=True)

        # Initialize components
        self.task_extractor = TaskExtractor()
        self.task_validator = TaskValidator()

        # Publisher for validated tasks
        self.task_publisher = rospy.Publisher("validated_tasks", Task, queue_size=10)

        # Subscriber for transcription input
        self.subscription = rospy.Subscriber(
            "transcription_topic", String, self.listener_callback, queue_size=10
        )

        rospy.loginfo("Task Validator Node is ready.")

    def listener_callback(self, msg):
        # Process the user input message
        user_input = msg.data
        rospy.loginfo(f"Received transcription: {user_input}")
        self.process_input(user_input)

    def process_input(self, user_input):
        # Extract tasks using the task extractor
        tasks = self.task_extractor.extract_tasks(user_input)
        rospy.loginfo(f"Extracted tasks: {tasks}")

        # Iterate through the extracted tasks and validate them
        for i, task_data in enumerate(tasks):
            try:
                # Convert the task data to a TaskInformation instance
                if isinstance(task_data, dict):
                    task = TaskInformation(**task_data)
                elif isinstance(task_data, TaskInformation):
                    task = task_data
                else:
                    rospy.logwarn(f"Unexpected task data type: {type(task_data)}")
                    continue
                rospy.loginfo(f"Processing task {i + 1}: {task}")
                # Validate the task
                is_valid = self.task_validator.validate_task(task, i)
                if is_valid:
                    rospy.loginfo(f"Task {i + 1} is valid: {task}")
                    self.publish_validated_task(task)
                else:
                    questions = self.task_validator.get_missing_info_questions(i)
                    rospy.loginfo(f"Task {i + 1} is missing information: {questions}")
            except Exception as e:
                rospy.logwarn(f"Failed to process task {i + 1}: {e}")

    def publish_validated_task(self, task: TaskInformation):
        # Create a Task message from TaskInformation and populate it
        task_msg = Task()
        task_msg.object_of_interest = task.object_of_interest or []
        task_msg.task_type = task.task_type or ''
        task_msg.source_location = task.source_location or []
        task_msg.target_location = task.target_location or []
        task_msg.context = task.context or ''

        # Publish the validated task
        try:
            self.task_publisher.publish(task_msg)
            rospy.loginfo(f"Published validated task: {task_msg}")
        except Exception as e:
            rospy.logwarn(f"Failed to publish validated task: {e}")


def main():
    try:
        node = TaskValidatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task Validator Node terminated.")


if __name__ == "__main__":
    main()
