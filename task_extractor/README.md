# Task Extractor Package

The `task_extractor` package is a ROS 2 package designed to process and validate task commands using natural language input, powered by a locally run LLM model through the Ollama framework. This package includes functionality to parse commands, validate extracted tasks, and publish them on a ROS topic for downstream processing. It also incorporates a custom interface for defining task messages.

## Features

- **Natural Language Parsing**: Parses natural language input to extract tasks using a custom extractor.
- **Task Validation**: Validates extracted tasks and checks for missing information.
- **Task Publication**: Publishes validated tasks to a ROS topic using a custom message format for downstream nodes.

## Package Dependencies

- `rclpy`: ROS 2 Python client library.
- `std_msgs`: Standard ROS 2 message types.
- `task_msgs`: Custom message package containing the `Task` message type used to represent tasks.
- `langchain` and `langchain-ollama`: Libraries used to interact with large language models (LLMs) for task extraction.
- `pydantic`: For data validation and parsing in Python.

## Prerequisites

- **Ollama Framework**: This package requires the Ollama framework to run a local LLM model. You can download Ollama from [Ollama's official website](https://ollama.com/).
- **LLM Model**: The package uses the `gemma2` model as the LLM model for task extraction. You’ll need to download `gemma2` within Ollama. To download and run the model use the following command:
  ```bash
    ollama run gemma2:9b
  ```
  Additionally the package was tested with these models:
  - `gemma2:2b` 
  - `llama3.2:1b` and `llama3.2:3b` models

- **Python Packages**: Install necessary Python packages via pip:

  ```bash
  pip install langchain-ollama langchain pydantic
  ```

## Installation

### 1. Clone the Repository

Clone this package into the `src` directory of your ROS 2 workspace:

### 2. Build the Package

Navigate to the root of your ROS 2 workspace and build the package:

```bash
cd ~/your_ros2_ws
colcon build --packages-select task_extractor
```

### 3. Source the Workspace

After building, source the workspace to make the package available:

```bash
source install/setup.bash
```

If using a virtual environment in your workspace for Python modules, activate it:

```bash
source ~/your_ros2_ws/venv/bin/activate
```

For ROS to recognize the virtual environment’s modules, set the `PYTHONPATH` variable to the virtual environment’s `site-packages` directory:

```bash
export PYTHONPATH=$PYTHONPATH:~/your_ros2_ws/venv/lib/python3.12/site-packages
```

## Usage

### Running the Task Validator Node

The primary node in this package is the `TaskValidatorNode`. This node listens for natural language commands on the `transcription_topic` topic, processes and validates the extracted tasks, and publishes validated tasks on the `validated_tasks` topic. It is designed to work with a transcription node from the `whisper_stt` package, which publishes transcriptions to `transcription_topic`.

To run the node:

```bash
ros2 run task_extractor task_validator_node
```

### Publishing Test Messages

You can publish a test message directly to the `transcription_topic` to simulate task input:

```bash
ros2 topic pub /transcription_topic std_msgs/msg/String "data: 'Go to the pantry, pick up a bottle of water and a snack, take them to the gym.'"
```

### Subscribing to Validated Tasks

To view the validated tasks published by the node, subscribe to the `validated_tasks` topic:

```bash
ros2 topic echo /validated_tasks
```

## Custom Message: `Task.msg`

The `Task` message, defined in the `task_msgs` package, represents each validated task. This message format includes fields for the object of interest, task type, source location, target location, and context.

**Message Definition:**

```plaintext
string[] object_of_interest
string task_type
string[] source_location
string[] target_location
string context
```

## Example Workflow

1. **Input Command**: "Go to the pantry, pick up a bottle of water and a snack, and take them to the gym."
2. **Extraction and Validation**: The `task_validator_node` processes the command, validates the tasks, and prepares them for publication.
3. **Publication**: Each validated task is published as a `Task` message on the `validated_tasks` topic.

## Contributing

Contributions are welcome! Please fork the repository and create a pull request to contribute improvements.

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contact

For questions or assistance, please contact [MarekC96](mailto:marek.cornak@stuba.sk).
```
