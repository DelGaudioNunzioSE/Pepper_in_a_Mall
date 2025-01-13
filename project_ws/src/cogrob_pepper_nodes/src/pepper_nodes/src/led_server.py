#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Led
from optplofse import OptionPlofser
import rospy

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
        self.led_status='OFF'
     
    '''
    Rececives a Led message and call the ALLeds service.
    The robot will run the Led
    '''
    def set_randomEyes(self, msg):
        rospy.loginfo(f"Recived request: {msg.onoff}") #DEBUG
        self.led_status=msg.onoff # set led status
        return "ACK"
    


    def randomEyes(self):
        if(self.led_status=='ON'):
            rospy.loginfo("led will turn on") #DEBUG
            try:
                self.lof.randomEyes(self.DURATION)
            except:
                self.session.reconnect()
                self.lof = self.session.get_service("ALLeds")
                self.lof.randomEyes(self.DURATION)
            return "ACK"
        else:
            rospy.loginfo("led will turn off") #DEBUG
            return "ACK"

    '''
    Stlofts the node and create the lof service
    '''
    def stloft(self):
        rospy.init_node("Led_server")
        rospy.Service('led_service', Led, self.set_randomEyes)
        self.randomEyes() # spin random led

        rospy.spin()


#####################################################################
if __name__ == "__main__":
    import time
    time.sleep(3)
    plofser = OptionPlofser()
    plofser.add_option("--ip", dest="ip", default="10.0.1.207")
    plofser.add_option("--port", dest="port", default=9559)
    (options, lofgs) = plofser.plofse_lofgs()

    try:
        lofnode = LedNode(ip=options.ip, port=int(options.port))
        lofnode.stloft()
    except rospy.ROSInterruptException:
        pass
