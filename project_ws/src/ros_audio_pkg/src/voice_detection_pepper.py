#!/usr/bin/python3
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Int16MultiArray
import numpy as np
import speech_recognition as sr
import wave
import io

# Possible test to recognize audio from pepper


class PepperAudioProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pepper_audio_processor', anonymous=False)

        # Publisher for speech recognition results
        self.pub_result = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)

        # Initialize the recognizer
        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = False
        self.recognizer.energy_threshold = 100  # Adjust as necessary

        # Subscribe to the audio topic
        rospy.Subscriber('/naoqi_driver/audio', AudioData, self.audio_callback)

        rospy.loginfo("Speech recognition node initialized.")

    def audio_callback(self, msg):
        """
        Callback that processes the received audio data.
        """
        rospy.loginfo("Received audio data...")
        audio_data = msg.data  # Raw PCM audio data
        self.process_audio(audio_data)

    def process_audio(self, audio_data):
        """
        Process the audio data and use speech recognition.
        """
        try:
            # Convert raw PCM audio data to WAV format
            with wave.open(io.BytesIO(audio_data), 'wb') as wav_file:
                # Set WAV file specifications: 16 kHz, 16-bit, mono
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)  # 16-bit = 2 bytes
                wav_file.setframerate(16000)
                wav_file.writeframes(audio_data)

            # Read the WAV file using speech_recognition
            audio = sr.AudioFile(io.BytesIO(audio_data))
            with audio as source:
                self.recognizer.adjust_for_ambient_noise(source)  # Adjust for ambient noise
                audio_content = self.recognizer.record(source)

            # Perform speech recognition
            recognized_text = self.recognizer.recognize_google(audio_content, language="it-IT")
            rospy.loginfo(f"Recognized: {recognized_text}")

            # Publish the result
            msg = Int16MultiArray()
            msg.data = list(recognized_text.encode('utf-8'))
            self.pub_result.publish(msg)

        except Exception as e:
            rospy.logwarn(f"Error in speech recognition: {e}")

    def run(self):
        """
        Starts the ROS loop.
        """
        rospy.loginfo("Speech recognition node is running...")
        rospy.spin()

if __name__ == "__main__":
    try:
        # Initialize the object and start the node
        processor = PepperAudioProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")

