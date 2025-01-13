#!/usr/bin/python3

import random
import rospy
from pepper_nodes.srv import Led, LedRequest, LedResponse
from std_msgs.msg import String

class Handler:
    '''
    The constructor creates the service proxy object, which is able to make the robot speak
    '''
    def __init__(self):
        self.lof = rospy.ServiceProxy("led_service", Led)

        

    '''
    This method calls the Text to Speech service and sends it the desired text to be played.
    text -> aniation to play
    '''
    
    def call_on(self, audio):
        msg = LedRequest()
        msg.onoff = 'ON' #se on
        rospy.loginfo("Request for led") #DEBUG
        resp = self.lof(msg)
        rospy.loginfo(resp.ack)


    def call_off(self, text):
        msg = LedRequest()
        msg.onoff = 'OFF' #se off
        rospy.loginfo("Request for led") #DEBUG
        resp = self.lof(msg)
        rospy.loginfo(resp.ack)





############################################################
if __name__ == "__main__":
    TOPIC_START='mic_data'
    TOPIC_STOP='bot_answer'
    NODE_NAME = "Led_node"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    # topic to read to know what Led do
    sub = rospy.Subscriber(TOPIC_START, String, handler.call_on)
    sub = rospy.Subscriber(TOPIC_STOP, String, handler.call_off)

