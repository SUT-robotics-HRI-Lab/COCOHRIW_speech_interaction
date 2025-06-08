# tts_service_node.py

import rclpy
from rclpy.node import Node
from coqui_tts_interfaces.srv import Speak
from coqui_tts_interfaces.msg import SpeakingStatus
from TTS.api import TTS
import sounddevice as sd
import numpy as np
import os
import threading
from std_msgs.msg import Bool
import time
import yaml
from ament_index_python.packages import get_package_share_directory


class TTSServiceNode(Node):
    def __init__(self):
        super().__init__('tts_service_node')

        # Find config.yaml in the package share directory
        package_share = get_package_share_directory('coqui_tts')
        config_path = os.path.join(package_share, 'config.yaml')
        self.get_logger().info(f"Looking for config.yaml at: {config_path}")
        if not os.path.exists(config_path):
            self.get_logger().warn(f"config.yaml not found at {config_path}. Using default parameters.")

        config = {
            "model_name": "tts_models/en/ljspeech/tacotron2-DDC",
            "device": "cuda", # or "cpu"
            "sample_rate": 22050,
            "service_name": "/coqui_tts/synthesize",
            "throttling_mode": "mutex"
        }

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

        self.model_name = config["model_name"]
        self.device = config["device"]
        self.sample_rate = config["sample_rate"]
        self.service_name = config["service_name"]
        self.throttling_mode = config["throttling_mode"]

        if self.throttling_mode not in ["mutex", "reject"]:
            raise ValueError(f"Invalid throttling_mode: '{self.throttling_mode}'")

        self.lock = threading.Lock()
        self.busy = False

        self.get_logger().info(f"Loading Coqui TTS model '{self.model_name}' on {self.device}")
        self.tts = TTS(model_name=self.model_name, progress_bar=False)
        self.tts.to(self.device)

        self.speaking_pub = self.create_publisher(SpeakingStatus, '/is_speaking', 10)

        self.srv = self.create_service(Speak, self.service_name, self.speak_callback)
        self.get_logger().info(f"TTS Service Node is ready. Listening on service: '{self.service_name}'")

    def speak_callback(self, request, response):
        if self.throttling_mode == "mutex":
            with self.lock:
                return self.handle_request(request, response)
        elif self.throttling_mode == "reject":
            if self.busy:
                response.success = False
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
            time.sleep(0.5)  # Allow Whisper node to react

            self.get_logger().info(f"Synthesizing speech for: {request.text}")
            audio = self.tts.tts(request.text)
            sd.play(np.array(audio), samplerate=self.sample_rate)
            sd.wait()

            self.speaking_pub.publish(SpeakingStatus(is_speaking=False, text=request.text))
            response.success = True
        except Exception as e:
            self.get_logger().error(f"TTS synthesis failed: {e}")
            self.speaking_pub.publish(SpeakingStatus(is_speaking=False, text=request.text))
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TTSServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
