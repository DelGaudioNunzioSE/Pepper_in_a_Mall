#!/usr/bin/python3

import random
import rospy
from pepper_nodes.srv import Animation, AnimationRequest, AnimationResponse
from std_msgs.msg import String

class Handler:
    '''
    The constructor creates the service proxy object, which is able to make the robot speak
    '''
    def __init__(self):
        self.ar = rospy.ServiceProxy("/animation_service", Animation)
        

    '''
    This method calls the Text to Speech service and sends it the desired text to be played.
    text -> aniation to play
    '''
    
    def call(self, text):
        msg = AnimationRequest()
        msg.speech = text.data #text.data
        action=self._detect_action(text)
        rospy.loginfo(f"Request for action: {action}") #DEBUG
        resp = self.ar(action)
        rospy.loginfo(resp.ack)

    # WE SHOULD IMPLEMENT A METHOD THAT GET FROM THE PHRASE THE ACTION
    def _detect_action(self,text):
        base_action="animations/Stand/BodyTalk/BodyTalk_"
        random_number=random.randint(1, 16)
        final_string = f"{base_action}{random_number}"
        return final_string




############################################################
if __name__ == "__main__":
    NODE_NAME = "animation_node"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    # topic to read to know what animation do
    sub = rospy.Subscriber("bot_answer", String, handler.call)

