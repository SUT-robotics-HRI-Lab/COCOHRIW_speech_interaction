import threading
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
import whisper
import spacy

# Load the Whisper model (medium English model)
model = whisper.load_model("medium.en", device="cuda:0")

# Load SpaCy English language model
nlp = spacy.load("en_core_web_sm")

# Audio recording parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
REALTIME_SECONDS = 6

audio = pyaudio.PyAudio()
transcription_queue = queue.Queue()  # Shared queue between threads


def record_audio_stream(stream):
    frames = []
    for _ in range(0, int(RATE / CHUNK * REALTIME_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
    return frames


def transcribe_audio(frames):
    try:
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / (1 << 15)
        result = model.transcribe(audio_data, language="en")
        print("Transcription:", result["text"])
        return result["text"]
    except Exception as e:
        return f"Error during transcription: {e}"


def validate_transcription(text):
    """Check if the transcription is likely meaningful using SpaCy."""
    doc = nlp(text)

    # Simple heuristic: check if the sentence has a subject and a verb
    #has_subject = any(token.dep_ == "nsubj" for token in doc)
    has_verb = any(token.pos_ == "VERB" for token in doc)
    #has_predicate = any(token.dep_ == "ROOT" for token in doc)

    #if has_subject and has_verb: #and has_predicate:
    if has_verb:
        return True
    return False


def listening_thread(stream):
    while True:
        frames = record_audio_stream(stream)
        transcription = transcribe_audio(frames)
        transcription_queue.put(transcription)


class TranscriptionNode(Node):
    def __init__(self):
        super().__init__('transcription_node')
        self.publisher_ = self.create_publisher(String, 'transcription_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_transcription)
        self.get_logger().info('Transcription Node has been started.')

    def publish_transcription(self):
        if not transcription_queue.empty():
            transcription = transcription_queue.get()
            if validate_transcription(transcription):
                self.get_logger().info(f'Valid transcription: {transcription}')
                msg = String()
                msg.data = transcription
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published transcription: {transcription}')
            else:
                self.get_logger().info('Transcription was invalid and not published.')


def main(args=None):
    rclpy.init(args=args)
    node = TranscriptionNode()

    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    listener_thread = threading.Thread(target=listening_thread, args=(stream,))
    listener_thread.daemon = True
    listener_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop_stream()
        stream.close()
        audio.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
