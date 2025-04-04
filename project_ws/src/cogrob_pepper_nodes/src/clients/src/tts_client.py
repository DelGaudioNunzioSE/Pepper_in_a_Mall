#!/usr/bin/python3

import rospy
from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse
from std_msgs.msg import String

class Handler:
    '''
    The constructor creates the service proxy object, which is able to make the robot speak
    '''
    def __init__(self):
        self.tts = rospy.ServiceProxy("tts", Text2Speech)
        

    '''
    This method calls the Text to Speech service and sends it the desired text to be played.
    '''
    def call(self, text):
        msg = Text2SpeechRequest()
        msg.speech = text.data #text.data
        rospy.loginfo(msg)
        resp = self.tts(msg)
        rospy.loginfo(resp.ack)

if __name__ == "__main__":
    NODE_NAME = "tts_node"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    sub = rospy.Subscriber("bot_answer", String, handler.call)
    rospy.spin()

