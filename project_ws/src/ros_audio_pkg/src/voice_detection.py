#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

import time
import speech_recognition as sr

HZ=44100 #44100 #16000
CHUNK=2048 #2048# 1024
threshold=300

class VoiceDetectionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('voice_detection_node', anonymous=False)

        # Publisher for microphone data (raw audio)
        self.pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)

        # Initialize recognizer for speech recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = False
        self.recognizer.energy_threshold=threshold #to set for evry mic
        

        # Set up the microphone (device_index should be set correctly depending on the system)
        self.microphone = sr.Microphone(device_index=4, sample_rate=HZ, chunk_size=CHUNK)


        # Start listening in the background and call the callback when audio is detected
        self.stop_listening = self.recognizer.listen_in_background(self.microphone, self.audio_callback)

        rospy.loginfo("Voice detection node initialized and listening...")

    def audio_callback(self, recognizer, audio):
        """
        This callback is called from the background thread when audio is detected.
        Converts the audio signal into an array of values and publishes it to the topic.
        """
        rospy.loginfo("Audio data received, processing...")
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)  # Convert audio data to numpy array
        data_to_send = Int16MultiArray()  # ROS message to send
        data_to_send.data = data  # Set the audio data in the message
        self.pub.publish(data_to_send)  # Publish to the topic

    def run(self):
        """
        Keeps the ROS node running.
        """
        rospy.spin()  # Keeps the node running and waiting for messages




##############################################################################
if __name__ == "__main__":
    try:
        # Create an instance of the VoiceDetectionNode and run the node
        voice_node = VoiceDetectionNode()
        voice_node.run()  # Start the ROS loop
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice detection node interrupted.")
