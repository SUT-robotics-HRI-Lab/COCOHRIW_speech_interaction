#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pyaudio
import numpy as np
import whisper
from langchain.prompts import PromptTemplate
from langchain_ollama import OllamaLLM
import threading
import queue

# Load the Whisper model
model = whisper.load_model("medium.en", device="cuda:0")

# Define the prompt template for the LLM check
prompt = PromptTemplate(
    input_variables=["user_input"],
    template=" Is the following text a request for someone to some task? '{user_input}' answer only Yes or No"
)

# Initialize the Ollama LLM
llm = OllamaLLM(model="gemma2:2b", temperature=0.1, device="cuda:0")


# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
REALTIME_SECONDS = 6

audio = pyaudio.PyAudio()
transcription_queue = queue.Queue()

def record_audio_stream(stream):
    """Record audio from the microphone."""
    frames = []
    for _ in range(0, int(RATE / CHUNK * REALTIME_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
    return frames

def transcribe_audio(frames):
    """Transcribe audio using Whisper."""
    try:
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / (1 << 15)
        result = model.transcribe(audio_data, language="en")
        rospy.loginfo(f"Transcription: {result['text']}")
        return result["text"]
    except Exception as e:
        return f"Error during transcription: {e}"

def validate_transcription(text):
    #if transcription is empty or one word, return False
    if not text or len(text.split()) < 2:
        return False

    llm_input = prompt.format(user_input=text)
    llm_response = llm.invoke(llm_input)
    rospy.loginfo(f"LLM response: {llm_response}")
    if "yes" in llm_response.lower():
        return True
    else:
        rospy.loginfo(f"LLM rejected transcription: {text}")
        return False

def listening_thread(stream):
    """Thread for capturing and transcribing audio."""
    while not rospy.is_shutdown():
        frames = record_audio_stream(stream)
        transcription = transcribe_audio(frames)
        transcription_queue.put(transcription)

def main():
    """Main function for the transcription node."""
    rospy.init_node("transcription_node")
    pub = rospy.Publisher("transcription_topic", String, queue_size=10)

    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    thread = threading.Thread(target=listening_thread, args=(stream,))
    thread.daemon = True
    thread.start()

    rate = rospy.Rate(1)  # Publish rate in Hz

    while not rospy.is_shutdown():
        if not transcription_queue.empty():
            transcription = transcription_queue.get()
            if validate_transcription(transcription):
                rospy.loginfo(f"Valid transcription: {transcription}")
                pub.publish(transcription)
            else:
                rospy.loginfo("Invalid transcription.")
        rate.sleep()

    stream.stop_stream()
    stream.close()
    audio.terminate()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
