#!/usr/bin/python3
from utils import Session
from optparse import OptionParser
import rospy
from std_msgs.msg import String
import time
from rasa_ros.srv import Dialogue, DialogueResponse

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
        self.ba = self.session.get_service("ALBasicAwareness")
        self.motion = self.session.get_service("ALMotion")
        self.motion.wakeUp()
        self.ba.setEngagementMode("FullyEngaged")
        self.ba.setEnabled(True)   
        self.em = self.session.get_service("ALMood")




    def start(self):
        rospy.init_node("AutoTrack_server")

        rospy.spin()


#####################################################################
if __name__ == "__main__":
    import time
    time.sleep(3)
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, bags) = parser.parse_args()

    try:
        banode = AutoTrackNode(ip=options.ip, port=int(options.port))
        banode.start()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
