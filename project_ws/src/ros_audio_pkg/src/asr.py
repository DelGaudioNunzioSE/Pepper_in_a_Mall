#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
import speech_recognition as sr
from difflib import get_close_matches

class SpeechRecognitionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('speech_recognition', anonymous=True)

        # Publishers
        self.pub_audio = rospy.Publisher('voice_data', Int16MultiArray, queue_size=5)
        self.pub_text = rospy.Publisher('voice_txt', String, queue_size=10)

        # Initialize recognizer
        self.recognizer = sr.Recognizer()

        # Sample rate and channels
        self.sample_rate = 44100
        self.channels = 2

        # Subscriber
        self.subscriber = rospy.Subscriber("mic_data", Int16MultiArray, self.audio_callback)

        rospy.loginfo("SpeechRecognitionNode initialized and listening to 'mic_data'.")




    def _word_replacement(self,testo, lista_parole=['person','man','woman','men','women','people','hat','hats','bag','bags','have','has','how', 'Pepper','what','where','at'],soglia=0.9):
        """
        Sostituisce le parole nella stringa con quelle più simili presenti in una lista, se la somiglianza supera una soglia.

        Args:
            testo (str): La stringa in cui effettuare le sostituzioni.
            lista_parole (list): Lista di parole accettabili per la sostituzione.
            soglia (float): Soglia di somiglianza (valore tra 0 e 1).

        Returns:
            str: La stringa modificata con le sostituzioni effettuate.
        """

        parole_nel_testo = testo.split()  # Dividi la stringa in parole
        parole_modificate = []

        for parola in parole_nel_testo:
            # Trova la parola più simile nella lista
            simili = get_close_matches(parola, lista_parole, n=1, cutoff=soglia)
            if simili:
                # Sostituisci con la parola più simile
                parole_modificate.append(simili[0])
            else:
                # Mantieni la parola originale se non ci sono corrispondenze
                parole_modificate.append(parola)

        # Ricostruisci la stringa
        return " ".join(parole_modificate)




    def audio_callback(self, audio_msg):
        """Callback for processing audio data."""
        try:
            # Convert ROS Int16MultiArray to NumPy array
            audio_data = np.array(audio_msg.data, dtype=np.int16)
            audio = sr.AudioData(audio_data.tobytes(), self.sample_rate, self.channels)

            # Recognize speech
            spoken_text = self.recognizer.recognize_google(audio, language='en-US')
            spoken_text.split()
            rospy.loginfo(f"Recognized text: {spoken_text}")
            new_spoken_text=self._word_replacement(f"{spoken_text}")
            rospy.loginfo(f"Recognized replaced text: {new_spoken_text}")

            # Publish audio and recognized text
            self.pub_audio.publish(audio_msg)
            self.pub_text.publish(new_spoken_text)

        except sr.UnknownValueError:
            
            rospy.logwarn("Google Speech Recognition could not understand the audio.")
        except sr.RequestError as e:
            rospy.logerr(f"Google Speech Recognition service error: {e}")
        except Exception as e:
            rospy.logerr(f"An error occurred in the audio callback: {e}")

    from difflib import get_close_matches




if __name__ == '__main__':
    rospy.loginfo("Starting SpeechRecognitionNode...")
    node = SpeechRecognitionNode()
    rospy.spin()