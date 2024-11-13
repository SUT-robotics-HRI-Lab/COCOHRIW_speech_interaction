# Task Interface Package (`task_msgs`)

The `task_msgs` package is a ROS 2 package that defines custom message types for representing tasks in the `task_extractor` package. The main message, `Task.msg`, is used to communicate validated tasks between nodes, enabling seamless task management and processing within a ROS 2 ecosystem.

## Message Definition

The primary message defined in this package is `Task.msg`. This message format is designed to represent various task parameters, including the objects involved, task type, source and target locations, and context. 

### `Task.msg`

The `Task.msg` message includes the following fields:

```plaintext
string[] object_of_interest      # List of objects involved in the task
string task_type                 # Type of task (e.g., navigate, inspect, relocate)
string[] source_location         # Source location(s) for the task
string[] target_location         # Target location(s) for the task
string context                   # Additional context or description of the task
```

This message structure is flexible, allowing multiple objects and locations to be specified as lists. It is designed to support various task types and provides a `context` field for additional details.

## Example Use Case

The `Task.msg` message is primarily used in the `task_extractor` package’s `TaskValidatorNode`. Here’s an example of how it might be used:

1. **Input Command**: "Go to the pantry, pick up a bottle of water and a snack, and take them to the gym."
2. **Task Validation and Processing**: The `task_validator_node` in the `task_extractor` package processes this command, extracting and validating each task.
3. **Task Publication**: Each validated task is published as a `Task` message on a ROS topic (e.g., `validated_tasks`), where other nodes can subscribe to receive and process the tasks.

## Installation

### 1. Clone the Repository

Clone this package into the `src` directory of your ROS 2 workspace.

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/yourusername/task_msgs.git
```

### 2. Build the Package

Navigate to the root of your ROS 2 workspace and build the package:

```bash
cd ~/your_ros2_ws
colcon build --packages-select task_msgs
```

### 3. Source the Workspace

After building, source the workspace:

```bash
source install/setup.bash
```

## Usage

The `Task.msg` message can be used in any ROS 2 node that requires task communication. Here’s an example of how to use it in a ROS 2 node:

### Publishing a Task Message

Here’s a sample Python snippet that publishes a `Task` message on the `validated_tasks` topic.

```python
import rclpy
from rclpy.node import Node
from task_msgs.msg import Task

class TaskPublisher(Node):
    def __init__(self):
        super().__init__('task_publisher')
        self.publisher_ = self.create_publisher(Task, 'validated_tasks', 10)
        self.publish_task()

    def publish_task(self):
        msg = Task()
        msg.object_of_interest = ['bottle of water', 'snack']
        msg.task_type = 'Relocate_Object'
        msg.source_location = ['pantry']
        msg.target_location = ['gym']
        msg.context = 'Pick up and move items to the gym'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published task: {msg}')

def main(args=None):
    rclpy.init(args=args)
    task_publisher = TaskPublisher()
    rclpy.spin(task_publisher)
    task_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscribing to a Task Message

Here’s an example of how to subscribe to the `Task` messages:

```python
import rclpy
from rclpy.node import Node
from task_msgs.msg import Task

class TaskSubscriber(Node):
    def __init__(self):
        super().__init__('task_subscriber')
        self.subscription = self.create_subscription(
            Task,
            'validated_tasks',
            self.task_callback,
            10)

    def task_callback(self, msg):
        self.get_logger().info(f'Received task: {msg}')

def main(args=None):
    rclpy.init(args=args)
    task_subscriber = TaskSubscriber()
    rclpy.spin(task_subscriber)
    task_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contact

For questions or assistance, please contact [MarekC96](mailto:marek.cornak@stuba.sk).
