# COCOHRIW 2: Collaborative Cognitive Human-Robot Interaction with Whisper

**COCOHRIW 2** is an advanced ROS 2-based framework for research in natural human-robot interaction. It combines automatic speech recognition, natural language understanding, and dialog management using large language models (LLMs). The system is designed to enable fluid spoken task-based interaction with robots, leveraging Whisper for transcription, LangChain+Ollama for LLM reasoning, and Coqui TTS for spoken feedback.

> **Note:** This work was funded and developed as part of the **European euROBIN project**.


## ✨ Features

* **Live voice command interpretation** using OpenAI Whisper
* **LLM-driven dialog classification and task extraction** using LangChain + Ollama
* **Clarification and feedback via speech** with Coqui TTS
* **Support for multi-turn interaction and banter**
* **Task validation and clarification using context memory**
* **Integration-ready framework for gesture and other modalities**
* **Modular architecture** supporting interchangeable ASR, LLM, and TTS models


## 🚀 Use Cases

* Voice-guided task execution for mobile or manipulator robots
* Interactive human-robot collaboration in research
* Study of fault-tolerant voice interaction and natural clarification

## ⚙️ Architecture Overview

* `transcription_node.py`: Captures and transcribes speech input using Whisper
* `transcription_classificator_node.py`: Uses SpaCy + LLM to classify input (task, clarification, banter)
* `task_extractor_node.py`: Converts valid task input to structured `Task` or `TaskArray` using a prompt
* `dialog_manager_node.py`: Manages the dialog flow and ensures tasks are complete
* `tts_service_node.py`: Speaks responses using Coqui TTS
* `llm_service_node.py`: Wraps LLM models (e.g., Gemma, LLaMA 3) via LangChain and Ollama

---

## 📊 Prerequisites

Install the following dependencies:

### ROS 2

* ROS 2 Jazzy (recommended)

### Python Dependencies

```bash
pip install pyaudio sounddevice numpy whisper langchain langchain_ollama spacy pydantic rapidfuzz webrtcvad
python -m spacy download en_core_web_sm
```

### Coqui TTS

```bash
coqui-tts #version that supports python =>3.12
```

### Ollama

* Install from [https://ollama.com](https://ollama.com)
* Download one of the tested models:

```bash
ollama pull gemma2  # gemma2:2b or llama3
```

---

## 🚧 Running the System

Start a ROS 2 environment and run the following nodes in **separate terminals** or with your preferred process manager:

```bash
# Terminal 1: Start LLM Service
ros2 run your_package llm_service_node.py

# Terminal 2: Start Coqui TTS Service
ros2 run your_package tts_service_node.py

# Terminal 3: Start Transcription Node
ros2 run your_package transcription_node.py

# Terminal 4: Start Classificator Node
ros2 run your_package transcription_classificator_node.py

# Terminal 5: Start Task Extractor
ros2 run your_package task_extractor_node.py

# Terminal 6: Start Dialog Manager
ros2 run your_package dialog_manager_node.py
```

---

## 🔗 Integration & Extension

* Compatible with gesture-based interaction pipelines
* Easily extendable to other multimodal channels
* Modular nodes allow rapid experimentation with new LLMs or dialog strategies
* **Configurable ASR**: Swap Whisper models like `base.en`, `tiny`, `medium.en`, etc.
* **Flexible LLM Backend**: Use different models downloaded from Ollama (e.g., `gemma`, `llama3`) or self-hosted models
* **Customizable TTS**: Choose from multiple voice models using the Coqui TTS library

---

## 🚀 EU Research Acknowledgment

This system is a product of the **euROBIN project**, a European network focused on advancing human-robot collaboration. Its development has been funded by the European Union and aims to promote reusable, interoperable solutions for cognitive robotics.

---

## 📄 License

MIT License

---

## 🚫 Disclaimer

This system is intended for **research purposes only**. Performance may vary based on environment, hardware, and chosen LLM.
---

## 📆 Future Work

* Launch file support
* Full gesture + speech fusion interface
---

For questions or collaboration, feel free to [open an issue](https://github.com/SUT-robotics-HRI-Lab/COCOHRIW_speech_interaction/issues) or contact the maintainers.
