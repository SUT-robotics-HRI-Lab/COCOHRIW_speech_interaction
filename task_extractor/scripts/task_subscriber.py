#!/usr/bin/env python

import rospy
from task_extractor.msg import Task  # Import the custom Task message

def task_callback(msg):
    """
    Callback function to process received Task messages.
    """
    rospy.loginfo("Received Task:")
    rospy.loginfo(f"  Object of Interest: {msg.object_of_interest}")
    rospy.loginfo(f"  Task Type: {msg.task_type}")
    rospy.loginfo(f"  Source Location: {msg.source_location}")
    rospy.loginfo(f"  Target Location: {msg.target_location}")
    rospy.loginfo(f"  Context: {msg.context}")

def main():
    """
    Main function to initialize the node and subscribe to the validated_tasks topic.
    """
    rospy.init_node("validated_task_subscriber", anonymous=True)
    rospy.Subscriber("validated_tasks", Task, task_callback)

    rospy.loginfo("Validated Task Subscriber is ready and listening...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Validated Task Subscriber terminated.")
