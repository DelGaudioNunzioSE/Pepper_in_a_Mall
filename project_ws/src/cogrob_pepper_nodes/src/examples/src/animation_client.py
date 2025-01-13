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
        self.ar = rospy.ServiceProxy("animation_service", Animation)
        

    '''
    This method calls the Text to Speech service and sends it the desired text to be played.
    text -> aniation to play
    '''
    
    def call(self, text):
        msg = AnimationRequest()
        action=self._detect_action(text.data) # evaluate action
        msg.action=action
        rospy.loginfo(f"Request for action: {msg.action}") #DEBUG
        resp = self.ar(msg)
        rospy.loginfo(resp.ack)

    # WE SHOULD IMPLEMENT A METHOD THAT GET FROM THE PHRASE THE ACTION
    def _detect_action(self,response):
        response=response.upper() # case insensitive
        if 'HELLO' in response:
            base_action="animations/Stand/Gestures/Hey_"
            random_number=3
            final_string = f"{base_action}{random_number}"

        elif 'BYE' in response or 'BOODBYE' in response:
            base_action="animations/Stand/Gestures/Hey_6"
            final_string = f"{base_action}"
            
        else:
            base_action="animations/Stand/BodyTalk/BodyTalk_"
            random_number=random.randint(1, 16)
            final_string = f"{base_action}{random_number}"
        return final_string
    
    def call_think(self, audio):
        msg = AnimationRequest()
        msg.action="animations/Stand/Gestures/Thinking_1"
        rospy.loginfo(f"Request for action: {msg.action}") #DEBUG
        resp = self.ar(msg)
        rospy.loginfo(resp.ack)


    def call_test(self, test):
        print('H1')
        msg = AnimationRequest()
        action=self._detect_action(test) # evaluate action
        msg.action=action
        rospy.loginfo(f"Request for action: {msg.action}") #DEBUG
        resp = self.ar(msg)
        rospy.loginfo(resp.ack)


############################################################
if __name__ == "__main__":
    NODE_NAME = "animation_node"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    # topic to read to know what animation do
    #handler.call_test('HELLO')
    sub = rospy.Subscriber("bot_answer", String, handler.call)
    sub = rospy.Subscriber("bot_answer", String, handler.call_think)

