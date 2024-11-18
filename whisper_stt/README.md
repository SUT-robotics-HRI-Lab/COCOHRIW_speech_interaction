# Whisper STT Package

The `whisper_stt` package is a ROS 2 package that transcribes real-time audio input using OpenAI’s Whisper model. It processes live microphone audio, validates the transcriptions for meaningfulness using both SpaCy for grammatical analysis and the Gemma 2 model for semantic validation, and publishes validated transcriptions to a ROS topic.

## Features

- **Real-Time Audio Transcription**: Uses the Whisper ASR model for continuous speech-to-text transcription.
- **Transcription Validation**: Ensures transcription accuracy and coherence with SpaCy’s grammatical analysis and further semantic validation through the Gemma 2 LLM model.
- **ROS Topic Publication**: Publishes validated transcriptions on a ROS topic for downstream processing in other ROS nodes.

## Package Dependencies

- `rclpy`: ROS 2 Python client library.
- `std_msgs`: Standard ROS 2 message types.

## Prerequisites

- **Ollama Framework**: This package uses the Ollama framework to run the local LLM model (Gemma 2). Download Ollama from [Ollama's official website](https://ollama.com/).
- **LLM Model**: The package requires the `gemma2` model to validate transcription meaning. Download `gemma2` using:
  ```bash
  ollama pull gemma2:2b
  ```
- **Python Packages**: Install necessary Python packages via pip:
  ```bash
  pip install openai-whisper spacy langchain langchain-ollama
  ```
  ```bash
  sudo apt-get install python3-pyaudio
  ```
  

  Also, download the English language model for SpaCy:
  ```bash
  python -m spacy download en_core_web_sm
  ```

## Installation

### 1. Clone the Repository

Clone this package into the `src` directory of your ROS 2 workspace:

```bash
cd ~/your_ros2_ws/src
git clone <repository_url> whisper_stt
```

### 2. Build the Package

Navigate to the root of your ROS 2 workspace and build the package:

```bash
cd ~/your_ros2_ws
colcon build --packages-select whisper_stt
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

Set the `PYTHONPATH` variable to the virtual environment’s `site-packages` directory:

```bash
export PYTHONPATH=$PYTHONPATH:~/your_ros2_ws/venv/lib/python3.x/site-packages
```

## Usage

### Running the Transcription Node

The primary node in this package is the `transcription_node`. This node captures real-time audio, transcribes it, validates the transcription, and publishes validated transcriptions on the `transcription_topic`.

To run the node:

```bash
ros2 run whisper_stt transcription_node
```

### Publishing Test Messages

You can publish a test message directly to the `transcription_topic` to simulate transcription output:

```bash
ros2 topic pub /transcription_topic std_msgs/msg/String "data: 'Test message'"
```

### Subscribing to Validated Transcriptions

To view the validated transcriptions published by the node, subscribe to the `transcription_topic`:

```bash
ros2 topic echo /transcription_topic
```

## Example Workflow

1. **Real-Time Audio Capture**: The `transcription_node` records audio segments in real-time from the microphone.
2. **Transcription with Whisper**: Each audio segment is transcribed by the Whisper model.
3. **Validation**: The transcription is validated for coherence and relevance using SpaCy and the Gemma 2 LLM.
4. **Publication**: Valid transcriptions are published to the `transcription_topic` for other nodes to use.

## Contributing

Contributions are welcome! Please fork the repository and create a pull request with any improvements.

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contact

For questions or assistance, please contact [MarekC96](mailto:marek.cornak@stuba.sk). 
