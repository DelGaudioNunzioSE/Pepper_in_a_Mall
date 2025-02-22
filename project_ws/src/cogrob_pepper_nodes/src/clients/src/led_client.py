#!/usr/bin/python3

import random
import rospy
from pepper_nodes.srv import Led, LedRequest, LedResponse
from std_msgs.msg import Int32
from std_msgs.msg import Int16MultiArray


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
        msg.time = "ON" #se on
        rospy.loginfo("Request for led") #DEBUG
        resp = self.lof(str(msg))
        rospy.loginfo(resp.ack)






############################################################
if __name__ == "__main__":
    TOPIC_START='mic_data'
    TOPIC_STOP='bot_answer'
    NODE_NAME = "Led_node"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    # topic to read to know what Led do
    sub = rospy.Subscriber(TOPIC_START, Int16MultiArray, handler.call_on)
    rospy.spin()

