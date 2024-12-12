#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String
from cogrob_pepper_nodes import Text2Speech, Text2SpeechRequest, Text2SpeechResponse




class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal

    '''

    def get_text(self):
        return input("[IN]:  ") 

    def set_text(self,text):
        print("[OUT]:",text)

dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
tts = rospy.ServiceProxy("/tts", Text2Speech)
terminal = TerminalInterface()

def callback(message):
    rospy.wait_for_service('dialogue_server') #blocca finchè il servizio non risponde
    message = message.data
    try:
        print("[IN]:", message)
        bot_answer = dialogue_service(message) #chiama il service dando in input il messaggio
        terminal.set_text(bot_answer.answer) #restituisce la risposta e lo stampa sulla shell
        call(bot_answer.answer)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call(self, text: str):
        msg = Text2SpeechRequest()
        msg.speech = text
        resp = self.tts(text) #chiama "say()"
        rospy.loginfo(resp.ack)

def main():
    rospy.init_node('writing')
    rospy.Subscriber("voice_txt", String, callback)
    rospy.spin()

   
if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
