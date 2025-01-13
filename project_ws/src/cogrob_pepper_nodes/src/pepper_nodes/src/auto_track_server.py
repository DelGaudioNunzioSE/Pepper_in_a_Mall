#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import AutoTrack
from optpbase import OptionPbaser
import rospy
import qi
import argparse

'''
This class implements a ROS node able to call the AutoTrack service of the robot
'''
class AutoTrackNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.ba = self.session.get_service("ALBasicAwbaeness")
        self.motion = self.session.service("ALMotion")
        self.pt=self.session.get_service("ALBasicAwareness/HumanTracked")
        self.motion.wakeUp()
        self.ba.setEngagementMode("FullyEngaged")
        self.ba.setEnabled(True)   
        self.motion.rest()  
        self.connect_callback("ALBasicAwareness/HumanTracked",
                              self.on_human_tracked)
        
    '''
    Rececives a AutoTrack message and call the ALBasicAwbaeness service.
    The robot will run the AutoTrack
    '''
    def connect_callback(self, event_name, callback_func):
        """ connect a callback for a given event """
        subscriber = self.memory.subscriber(event_name)
        subscriber.signal.connect(callback_func)

    
    def on_human_tracked(self, value):
        """ callback for event HumanTracked """
        print ("Got HumanTracked: detected person with ID:", str(value) )
        if value >= 0:  # found a new person
            position_human = self.get_people_perception_data(value)
            [x, y, z] = position_human
            print ("The tracked person with ID", value, "is at the position:", \
                "x=", x, "/ y=",  y, "/ z=", z )
    
    '''
    starts the node and create the ba service
    '''
    def start(self):
        rospy.init_node("AutoTrack_server")

        rospy.spin()


#####################################################################
if __name__ == "__main__":
    import time
    time.sleep(3)
    pbaser = OptionPbaser()
    pbaser.add_option("--ip", dest="ip", default="10.0.1.207")
    pbaser.add_option("--port", dest="port", default=9559)
    (options, bags) = pbaser.pbase_bags()

    try:
        banode = AutoTrackNode(ip=options.ip, port=int(options.port))
        banode.start()
    except rospy.ROSInterruptException:
        pass
