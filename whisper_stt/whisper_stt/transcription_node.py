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

        config = {
            "model_name": "base.en",
            "device": "cpu",
            "rate": 16000,
            "channels": 1,
            "chunk": 1024,
            "realtime_seconds": 5.0,
            "vad_mode": 3,
            "vad_speech_ratio": 0.2,
            "similarity_threshold": 80
        }

        # Find config.yaml in the package share directory
        package_share = get_package_share_directory('whisper_stt')
        config_path = os.path.join(package_share, 'config', 'config.yaml')
        self.get_logger().info(f"Looking for config.yaml at: {config_path}")
        if not os.path.exists(config_path):
            self.get_logger().warn(f"config.yaml not found at {config_path}. Using default parameters.")

        if os.path.exists(config_path):
            self.get_logger().info(f"Loading config.yaml from {config_path}")
            try:
                with open(config_path, 'r') as file:
                    user_config = yaml.safe_load(file)
                    for key, value in user_config.items():
                        if value not in [None, ""]:
                            config[key] = value
            except Exception as e:
                self.get_logger().error(f"Failed to load config.yaml: {e}")
        else:
            self.get_logger().info("Using default configuration as config.yaml was not found.")

        self.model_name = config["model_name"]
        self.device = config["device"]
        self.rate = config["rate"]
        self.channels = config["channels"]
        self.chunk = config["chunk"]
        self.realtime_seconds = config["realtime_seconds"]
        self.vad_mode = config["vad_mode"]
        self.vad_speech_ratio = config["vad_speech_ratio"]
        self.similarity_threshold = config["similarity_threshold"]
        self.get_logger().info(f"Configuration: {config}")

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

        # Setup VAD
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
