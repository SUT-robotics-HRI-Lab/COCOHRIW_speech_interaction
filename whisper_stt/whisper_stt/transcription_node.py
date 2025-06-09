# transcription_node.py

import threading
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from coqui_tts_interfaces.msg import SpeakingStatus
import pyaudio
import numpy as np
import whisper
import time
import webrtcvad
import collections
from rapidfuzz import fuzz
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class TranscriptionNode(Node):
    def __init__(self):
        super().__init__('transcription_node')

        # === Load defaults from YAML ===
        defaults = {
            "model_name": "tiny.en",
            "device": "cuda:0",
            "realtime_seconds": 2.5,
            "rate": 16000,
            "chunk": 1024,
            "channels": 1,
            "vad_mode": 3,
            "vad_speech_ratio": 0.25,
            "similarity_threshold": 85.0
        }

        package_share = get_package_share_directory('whisper_stt')
        config_path = os.path.join(package_share, 'config', 'config.yaml')
        self.get_logger().info(f"Looking for config.yaml at: {config_path}")

        if os.path.exists(config_path):
            self.get_logger().info(f"Loading config.yaml from {config_path}")
            try:
                with open(config_path, 'r') as file:
                    user_config = yaml.safe_load(file)
                    for key, value in user_config.items():
                        if value not in [None, ""]:
                            defaults[key] = value
            except Exception as e:
                self.get_logger().error(f"Failed to load config.yaml: {e}")
        else:
            self.get_logger().warn("config.yaml not found. Using built-in defaults.")

        # === Declare as ROS parameters ===
        for key, value in defaults.items():
            self.declare_parameter(key, value)

        # === Read parameters from node ===
        self.model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.realtime_seconds = self.get_parameter("realtime_seconds").get_parameter_value().double_value
        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        self.chunk = self.get_parameter("chunk").get_parameter_value().integer_value
        self.channels = self.get_parameter("channels").get_parameter_value().integer_value
        self.vad_mode = self.get_parameter("vad_mode").get_parameter_value().integer_value
        self.vad_speech_ratio = self.get_parameter("vad_speech_ratio").get_parameter_value().double_value
        self.similarity_threshold = self.get_parameter("similarity_threshold").get_parameter_value().double_value

        self.get_logger().info("Effective parameters:")
        self.get_logger().info(f"model_name: {self.model_name}, device: {self.device}, realtime_seconds: {self.realtime_seconds}")
        self.get_logger().info(f"rate: {self.rate}, chunk: {self.chunk}, channels: {self.channels}")
        self.get_logger().info(f"vad_mode: {self.vad_mode}, vad_speech_ratio: {self.vad_speech_ratio}")
        self.get_logger().info(f"similarity_threshold: {self.similarity_threshold}")

        # === Audio and VAD setup ===
        self.is_speaking = False
        self.last_spoken_text = ""
        self.create_subscription(SpeakingStatus, '/is_speaking', self.speaking_callback, 10)

        self.get_logger().info(f"Loading Whisper model: {self.model_name} on {self.device}")
        self.model = whisper.load_model(self.model_name, device=self.device)

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        self.transcription_queue = queue.Queue()
        self.publisher_ = self.create_publisher(String, '/transcription', 10)
        self.timer = self.create_timer(1.0, self.publish_transcription)
        self.get_logger().info('Whisper Transcription Node started.')

        # VAD setup
        self.vad = webrtcvad.Vad(self.vad_mode)
        self.frame_duration_ms = 30
        self.frame_bytes = int(self.rate * self.frame_duration_ms / 1000) * 2
        self.vad_buffer = collections.deque(maxlen=int(3000 / self.frame_duration_ms))

        self.start_listening_thread()

    def speaking_callback(self, msg: SpeakingStatus):
        self.is_speaking = msg.is_speaking
        self.last_spoken_text = msg.text.strip().lower()
        if msg.is_speaking:
            self.get_logger().info("TTS is speaking. Pausing transcription and clearing queue.")
            with self.transcription_queue.mutex:
                self.transcription_queue.queue.clear()
        else:
            self.get_logger().info("TTS stopped speaking. Resuming transcription.")
            self.get_logger().info(f"Last spoken text: '{self.last_spoken_text}'")
            time.sleep(1.5)

    def start_listening_thread(self):
        listener_thread = threading.Thread(target=self.listen_loop)
        listener_thread.daemon = True
        listener_thread.start()

    def listen_loop(self):
        while rclpy.ok():
            if self.is_speaking:
                time.sleep(0.1)
                continue

            raw_frames = []
            active_frames = 0
            total_frames = 0

            for _ in range(int(self.realtime_seconds * 1000 / self.frame_duration_ms)):
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                raw_frames.append(data)

                self.vad_buffer.append(data)
                total_frames += 1
                if len(data) >= self.frame_bytes:
                    chunk = data[:self.frame_bytes]
                    if self.vad.is_speech(chunk, self.rate):
                        active_frames += 1

            if total_frames == 0 or (active_frames / total_frames < self.vad_speech_ratio):
                self.get_logger().info("VAD: Skipping due to low speech activity.")
                continue

            audio_data = np.frombuffer(b''.join(raw_frames), dtype=np.int16)
            audio_data = audio_data.astype(np.float32) / (1 << 15)

            try:
                result = self.model.transcribe(audio_data, language="en", condition_on_previous_text=False, temperature=0.0)
                transcription = result["text"]
            except Exception as e:
                transcription = f"Error: {e}"

            self.transcription_queue.put(transcription.strip())

    def publish_transcription(self):
        if self.is_speaking:
            return

        if not self.transcription_queue.empty():
            transcription = self.transcription_queue.get().strip().lower()
            if not transcription:
                return

            if self.last_spoken_text:
                similarity = fuzz.partial_ratio(transcription, self.last_spoken_text)
                if similarity > self.similarity_threshold:
                    self.get_logger().info(f"Filtered TTS echo (similarity={similarity}): '{transcription}'")
                    return
                elif self.last_spoken_text in transcription:
                    transcription = transcription.replace(self.last_spoken_text, "").strip()
                    self.get_logger().info(f"Trimmed echo from transcription → '{transcription}'")
                else:
                    self.get_logger().info(f"Transcription differs from last spoken text (similarity={similarity}): '{transcription}'")

            if transcription:
                self.get_logger().info(f'Publishing: "{transcription}"')
                msg = String()
                msg.data = transcription
                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TranscriptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop_stream()
        node.stream.close()
        node.audio.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
