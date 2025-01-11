#!/usr/bin/python3


import rospy
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray

class HeadYawPitch:
    def __init__(self):
        # Definisci i publisher per il movimento della testa
        self.head_motion_yaw_pub = rospy.Publisher("/head_rotation/yaw", Float32MultiArray, queue_size=10)
        self.head_motion_pitch_pub = rospy.Publisher("/head_rotation/pitch", Float32MultiArray, queue_size=10)

        # Sottoscrizione alla topic "detection" per ricevere i rilevamenti
        rospy.Subscriber("detection", Detection2DArray, self.detection_callback)


    def calculate_head_angles(self, detection_box, image_width, image_height):
        """
        Calcola gli angoli di yaw e pitch in base alla posizione del bounding box.
        """
        center_x = (detection_box[1] + detection_box[3]) / 2
        center_y = (detection_box[0] + detection_box[2]) / 2

        # Normalizza il centro del bounding box in un range di [-1, 1]
        yaw = (center_x - image_width / 2) / (image_width / 2)
        pitch = (center_y - image_height / 2) / (image_height / 2)

        return yaw, pitch

    def detection_callback(self, msg):
        """
        Callback per i messaggi di detection, usata per controllare il movimento della testa.
        """
        if msg.detections:
            detection = msg.detections[0]
            bbox = detection.bbox
            yaw, pitch = self.calculate_head_angles(
                [bbox.center.x, bbox.center.y, bbox.size_x, bbox.size_y], 2592, 1944)  # Adatta la risoluzione dell'immagine

            velocity = 0.15
            # Creazione dei messaggi per yaw e pitch
            yaw_msg = Float32MultiArray()
            yaw_msg.data = [yaw, velocity]

            pitch_msg = Float32MultiArray()
            pitch_msg.data = [pitch, velocity]

            # Pubblica i messaggi per il movimento della testa
            self.head_motion_yaw_pub.publish(yaw_msg)
            self.head_motion_pitch_pub.publish(pitch_msg)

            rospy.loginfo(f"Head Yaw: {yaw}, Head Pitch: {pitch}")
    



if __name__ == "__main__":    
    handler = HeadYawPitch()
    rospy.init_node("head_yawpitch_node")
    rospy.spin()    

