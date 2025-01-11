#!/usr/bin/python3

from utils import Session
from optparse import OptionParser
from pepper_nodes.srv import WakeUp
import qi
import argparse
import sys
import rospy


class TrackNode:

    def __init__(self,ip,port,faceSize):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.faceSize = faceSize
        self.motion_proxy = self.session.get_service("ALMotion")
        self.posture_proxy = self.session.get_service("ALRobotPosture")
        self.tracker_service = self.session.get_service("ALTracker")
        self.animation_player_service = self.session.get_service("ALAnimationPlayer")

    def trackernode(self, *args):
        """
        Metodo per settare la faccia come target del tracking e la dimensione. Nel caso in cui si entri nell'except, si cerca di reinizializzare i service.
        """
        try:
            targetName = "Face"
            faceWidth = self.faceSize
            self.tracker_service_registerTarget(targetName,faceWidth)
            self.tracker_service.track(targetName)
        except:
            self.motion_proxy = self.session.get_service("ALMotion")
            self.posture_proxy = self.session.get_service("ALRobotPosture")
            self.tracker_service = self.session.get_service("ALTracker")
            self.animation_player_service = self.session.get_service("ALAnimationPlayer")

    def start(self):
        """
        Questo metodo fa partire il tracking
        """
        rospy.init_node("tracker_node")
        self.trackernode()
        rospy.Service("tracker", WakeUp, self.trackernode)
        rospy.spin()
    
    def stop(self):
        """
        Questo metodo ferma il tracking facendo tornare il robot in posizione di riposo. Nel caso in cui si entri nell' except, si cerca di ripristinare il tracking
        prima di tentare di fermarlo nuovamente
        """
        try:
            self.motion_proxy.rest()
            self.tracker_service.stopTracker()
        except:
            self.tracker_service = self.session.get_service("ALTracker")
            self.tracker_service.stopTracker()



if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.230") #da modificare con l'id di pepper da usare
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = TrackNode(options.ip, int(options.port), 0.1)
        node.start()
    except rospy.ROSInterruptException:
        node.stop()

    
        
