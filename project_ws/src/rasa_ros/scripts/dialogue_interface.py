#!/usr/bin/env python3

import time
import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import String
#from rasa_ros.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse
import os
from rasa_ros.srv import Dialogue, DialogueResponse
import string



class DialogueInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal

    '''

    def __init__(self):
        #self.tts = rospy.ServiceProxy("/tts", Text2Speech) #In questo modo accoppio il service e l'interfaccia
        #decido di fare un publisher in modo che la risposta di rasa possa essere usata da 
        #tutti i nodi di Pepper singolarmente, senza dover essere inseriti in questo package.
        self.dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
        self.pub = rospy.Publisher('bot_answer', String, queue_size=10)
        self.last_answer=""

    def get_text(self):
        return input("[IN]:  ") 

    def set_text(self,text):
        print("[OUT]:",text)

    def callback(self, message):
        rospy.wait_for_service('dialogue_server') #blocca finchè il servizio non risponde
        message = message.data
        try:
            print("[IN]:", message)

            if (self.jaccard_similarity(message,self.last_answer) >0.8):
                rospy.loginfo(f"Input: {message}", f"Last:{self.last_answer}")
                rospy.loginfo("auto listened")
                return

            # Response
            bot_answer = self.dialogue_service(message) #chiama il service dando in input il messaggio
            if(bot_answer.answer==""):
                bot_answer.answer = "I didn't understand, can you repeat?"

            self.set_text(bot_answer.answer) #restituisce la risposta e lo stampa sulla shell
            self.pub.publish(bot_answer.answer) #chiama il nodo tts e fa parlare
            self.last_answer=bot_answer.answer
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    

    def jaccard_similarity(s1: str, s2: str) -> float:
        """
        Calcola la somiglianza di Jaccard tra due frasi s1 e s2.
        
        :param s1: Prima frase.
        :param s2: Seconda frase.
        :return: Punteggio di somiglianza Jaccard (tra 0 e 1).
        """
        # Converto le frasi in insiemi di parole
        set_s1 = set(s1.lower().split())
        set_s2 = set(s2.lower().split())

        # Calcolo l'intersezione e l'unione
        intersection = set_s1.intersection(set_s2)
        union = set_s1.union(set_s2)

        # Calcolo la somiglianza di Jaccard
        similarity = len(intersection) / len(union)
        return similarity


def main():

    interface = DialogueInterface()
    rospy.init_node('writing')
    rospy.Subscriber("voice_txt", String, interface.callback)
    interface.pub.publish("Hello! How can i help you?") #da sbloccare con Pepper
    interface.set_text("Hello! How can i help you?")
    rospy.spin()

   
if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
