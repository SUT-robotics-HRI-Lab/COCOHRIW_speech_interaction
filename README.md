# COCOHRIW
This Repository contains ROS2 packages used in a project of Complex Collaborative HRI Workplace (COCOHRIW), which is a research project of Human-Robot Interaction methods conducted at the robotics department of Slovak University of Technology.

The project focuses on the interaction with a robotic system using natural means of human communication.

# Contents
The repository currently contains following packages:
 - stt_whisper
 - task_extractor
 - task_msgs

**stt_whisper package**
This ROS 2 package that transcribes real-time audio input using OpenAI’s Whisper model. It processes live microphone audio, validates the transcriptions for meaningfulness using both SpaCy for grammatical analysis and the Gemma 2 model for semantic validation, and publishes validated transcriptions to a ROS topic.

**task_extractor package**
This package is a ROS 2 package designed to process and validate task commands using natural language input, powered by a locally run LLM model through the Ollama framework. This package includes functionality to parse commands, validate extracted tasks, and publish them on a ROS topic for downstream processing. It also incorporates a custom interface for defining task messages.

**task_msgs package**
This package is a ROS 2 package that defines custom message types for representing tasks in the task_extractor package. The main message, Task.msg, is used to communicate validated tasks between nodes, enabling seamless task management and processing within a ROS 2 ecosystem.
