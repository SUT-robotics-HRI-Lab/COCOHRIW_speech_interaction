import rclpy
from rclpy.node import Node
from coqui_tts_interfaces.srv import Speak
from coqui_tts_interfaces.msg import SpeakingStatus
from TTS.api import TTS
import sounddevice as sd
import numpy as np
import os
import threading
import time
import yaml
import soundfile as sf
from tempfile import NamedTemporaryFile
from ament_index_python.packages import get_package_share_directory


class TTSServiceNode(Node):
    def __init__(self):
        super().__init__('tts_service_node')

        # Load config.yaml from package share
        package_share = get_package_share_directory('coqui_tts')
        config_path = os.path.join(package_share, 'config.yaml')
        self.get_logger().info(f"Looking for config.yaml at: {config_path}")

        config = {
            "model_name": "tts_models/en/ljspeech/tacotron2-DDC",
            "device": "cpu",
            "sample_rate": 22050,
            "service_name": "/coqui_tts/synthesize",
            "throttling_mode": "mutex",
            "is_multispeaker": False,
            "default_language": "en",
            "default_speaker_wav": "",
            "default_speaker_id": ""  # for built-in speaker voices
        }

        if os.path.exists(config_path):
            self.get_logger().info("Loading config.yaml...")
            try:
                with open(config_path, 'r') as file:
                    user_config = yaml.safe_load(file)
                    for key, value in user_config.items():
                        if value not in [None, ""]:
                            config[key] = value
            except Exception as e:
                self.get_logger().error(f"Failed to load config.yaml: {e}")

        # Assign config values
        self.model_name = config["model_name"]
        self.device = config["device"]
        self.sample_rate = config["sample_rate"]
        self.service_name = config["service_name"]
        self.throttling_mode = config["throttling_mode"]
        self.is_multispeaker = config["is_multispeaker"]
        self.default_language = config["default_language"]
        self.default_speaker_wav = config["default_speaker_wav"]
        self.default_speaker_id = config["default_speaker_id"]

        if self.throttling_mode not in ["mutex", "reject"]:
            raise ValueError(f"Invalid throttling_mode: '{self.throttling_mode}'")

        self.lock = threading.Lock()
        self.busy = False

        self.get_logger().info(f"Loading TTS model '{self.model_name}' on {self.device}")
        self.tts = TTS(model_name=self.model_name, progress_bar=False)
        self.tts.to(self.device)

        self.speaking_pub = self.create_publisher(SpeakingStatus, '/is_speaking', 10)
        self.srv = self.create_service(Speak, self.service_name, self.speak_callback)
        self.get_logger().info(f"TTS Service ready at '{self.service_name}'")

    def speak_callback(self, request, response):
        if self.throttling_mode == "mutex":
            with self.lock:
                return self.handle_request(request, response)
        elif self.throttling_mode == "reject":
            if self.busy:
                response.success = False
                response.filepath = ""
                return response
            else:
                self.busy = True
                try:
                    return self.handle_request(request, response)
                finally:
                    self.busy = False

    def handle_request(self, request, response):
        try:
            self.speaking_pub.publish(SpeakingStatus(is_speaking=True, text=request.text))
            time.sleep(0.5)

            if self.is_multispeaker:
                args = {
                    "text": request.text,
                    "language": self.default_language
                }

                if self.default_speaker_wav and os.path.exists(self.default_speaker_wav):
                    self.get_logger().info(f"[XTTS] Using speaker_wav: {self.default_speaker_wav}")
                    args["speaker_wav"] = self.default_speaker_wav
                elif self.default_speaker_id:
                    self.get_logger().info(f"[XTTS] Using speaker: {self.default_speaker_id}")
                    args["speaker"] = self.default_speaker_id  # ✅ FIXED here
                else:
                    raise ValueError("No valid speaker_wav or speaker_id set for multispeaker model.")

                audio = self.tts.tts(**args)
            else:
                self.get_logger().info(f"[Single-speaker] Synthesizing: '{request.text}'")
                audio = self.tts.tts(text=request.text)

            sd.play(np.array(audio), samplerate=self.sample_rate)
            sd.wait()

            with NamedTemporaryFile(delete=False, suffix=".wav") as f:
                sf.write(f.name, audio, self.sample_rate)
                response.filepath = f.name

            self.speaking_pub.publish(SpeakingStatus(is_speaking=False, text=request.text))
            response.success = True

        except Exception as e:
            self.get_logger().error(f"TTS synthesis failed: {e}")
            self.speaking_pub.publish(SpeakingStatus(is_speaking=False, text=request.text))
            response.success = False
            response.filepath = ""

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TTSServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
