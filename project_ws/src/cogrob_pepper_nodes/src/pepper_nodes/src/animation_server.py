#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Animation
from optparse import OptionParser
import rospy

'''
This class implements a ROS node able to call the Animation service of the robot
'''
class AnimationNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.ar = self.session.get_service("ALAnimationPlayer")
     
    '''
    Rececives a Animation message and call the ALAnimationPlayer service.
    The robot will run the animation
    '''
    def run(self, msg):
        rospy.loginfo(f"Recived request for action: {msg.action}") #DEBUG
        try:
            self.ar.run(msg.action)
        except:
            self.session.reconnect()
            self.ar = self.session.get_service("ALAnimationPlayer")
            self.ar.run(msg.action)
        return "ACK"
    
    '''
    Starts the node and create the ar service
    '''
    def start(self):
        rospy.init_node("animation_server")
        rospy.Service('animation_service', Animation, self.run)

        rospy.spin()


#####################################################################
if __name__ == "__main__":
    import time
    time.sleep(3)
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        animation = AnimationNode(ip=options.ip, port=int(options.port))
        animation.start()
    except rospy.ROSInterruptException:
        pass
