import threading
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import numpy as np
import time
import whisper
import sounddevice as sd

# Load the Whisper model (small version)
model = whisper.load_model("medium.en")

# Parameters for audio recording
FORMAT = pyaudio.paInt16  # Audio format
CHANNELS = 1              # Number of channels
RATE = 16000              # Sampling rate
CHUNK = 1024              # Buffer size
REALTIME_SECONDS = 5      # Duration of each recording segment

# Initialize PyAudio
audio = pyaudio.PyAudio()

# Shared queue for communication between threads
transcription_queue = queue.Queue()

# Function to record audio from the microphone in real-time
def record_audio_stream(stream):
    #print("Recording... Press Ctrl+C to stop.")

    frames = []

    for _ in range(0, int(RATE / CHUNK * REALTIME_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    # Save the recorded audio to a temporary WAV file
    wf = wave.open("temp.wav", 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(audio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    # Transcribe the recorded audio
    transcription = transcribe("temp.wav")
    print("Transcription:", transcription)
    
    time.sleep(0.1)  # Slight delay to prevent overlapping
    #print("Stopped recording.")

    return transcription

# Function to read audio file and preprocess
def read_audio(file_path):
    try:
        with wave.open(file_path, 'rb') as wf:
            n_frames = wf.getnframes()
            audio_data = np.frombuffer(wf.readframes(n_frames), dtype=np.int16)
            audio_data = audio_data.astype(np.float32) / (1 << 15)  # Normalize audio data
            return audio_data
    except Exception as e:
        print(f"Error reading audio file: {e}")
        return None

# Function to transcribe speech to text
def transcribe(file_path):
    audio_data = read_audio(file_path)
    if audio_data is None:
        return "Error reading audio file."

    try:
        # Whisper expects a float32 numpy array normalized to -1 to 1
        audio_data = audio_data / np.max(np.abs(audio_data))
        result = model.transcribe(audio_data, language="en")#, fp16=False)
        return result["text"]
    except Exception as e:
        return f"Error transcribing audio: {e}"

# Thread function for continuous listening and transcription
def listening_thread(stream):
    while True:
        transcription = record_audio_stream(stream)
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
            msg = String()
            msg.data = transcription
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published transcription: {transcription}')

def main(args=None):
    rclpy.init(args=args)
    node = TranscriptionNode()

    # Open the audio stream
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        frames_per_buffer=CHUNK)

    # Create and start the listening thread
    print
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