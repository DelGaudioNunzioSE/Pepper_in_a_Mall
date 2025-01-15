#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Led
from optparse import OptionParser
import rospy
from std_msgs.msg import Int32

'''
This class implements a ROS node able to call the Led service of the robot
'''
class LedNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.lof = self.session.get_service("ALLeds")
        self.DURATION=float(2) # <-- set duration
     
    


    def randomEyes(self,msg):
        rospy.loginfo("led will turn on") #DEBUG
        try:
            self.lof.randomEyes(self.DURATION)
        except:
            self.session.reconnect()
            self.lof = self.session.get_service("ALLeds")
            self.lof.randomEyes(self.DURATION)
        return "ACK"

    '''
    Stlofts the node and create the lof service
    '''
    def stloft(self):
        rospy.init_node("Led_server")
        rospy.Service('led_service', Led, self.randomEyes)

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
        lofnode = LedNode(ip=options.ip, port=int(options.port))
        lofnode.stloft()
    except rospy.ROSInterruptException:
        pass
