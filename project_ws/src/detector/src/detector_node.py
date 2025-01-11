#!/usr/bin/env python3
import os
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from detector import Detector
from std_msgs.msg import Float32MultiArray
import ros_numpy # pip3 install git+https://github.com/eric-wieser/ros_numpy

DET_PATH=os.path.join(os.path.dirname(__file__),'efficientdet_d1_coco17_tpu-32')
mydetector = Detector(DET_PATH)

rospy.init_node('detector_node')
pub = rospy.Publisher('detection', Detection2DArray, queue_size=2)

# Publishers for head yaw and pitch
head_reset_yaw = rospy.Publisher("/head_rotation/yaw", Float32MultiArray, queue_size=10)
head_reset_pitch = rospy.Publisher("/head_rotation/pitch", Float32MultiArray, queue_size=10)

# Detection message publisher
pub = rospy.Publisher('detection', Detection2DArray, queue_size=2)

# Funzione per calcolare gli angoli
def calculate_head_angles(detection_box, image_width, image_height):
    """
    Calculate yaw and pitch angles for head movement based on bounding box center.
    """
    # Considerando la persona al centro del bounding
    center_x = (detection_box[1] + detection_box[3]) / 2
    center_y = (detection_box[0] + detection_box[2]) / 2

    # Normalize center to the range of [-1, 1] where 0 is the center of the image
    yaw = (center_x - image_width / 2) / (image_width / 2)
    pitch = (center_y - image_height / 2) / (image_height / 2)

    return yaw, pitch


# Callback
def detection_callback(msg):
    """
    Callback for the detection message, used to control head rotation.
    """
    # Prende la prima detection (penso da modificare con chi parla)
    if msg.detections:
        detection = msg.detections[0]
        # Bounding box della prima persona tracciata 
        bbox = detection.bbox
        # Calcola yaw e pitch
        yaw, pitch = calculate_head_angles(
            [bbox.center.x, bbox.center.y, bbox.size_x, bbox.size_y], 2592, 1944)  #Immagine di 640x480 ma bisogna vedere la risoluzione di pepper
        
        velocity = 0.15
        # Creazione dei msg contenenti yaw, pitch e la velocita'
        yaw_msg = Float32MultiArray
        yaw_msg.data = [yaw, velocity]

        pitch_msg = Float32MultiArray
        pitch_msg.data = [pitch, velocity]
        # Publish per pubblicare yaw e pitch 
        head_reset_yaw.publish(yaw_msg)
        head_reset_pitch.publish(pitch_msg)

        rospy.loginfo(f"Head Yaw: {yaw}, Head Pitch: {pitch}")



def rcv_image(msg):
    image = ros_numpy.numpify(msg)
    detections = mydetector(image)
    message = Detection2DArray()
    for clabel,score,box in zip(detections['detection_classes'], detections['detection_scores'], detections['detection_boxes']):
        d = Detection2D()
        d.bbox.size_x = box[3]-box[1]
        d.bbox.size_y = box[2]-box[0]
        d.bbox.center.x = box[1]+d.bbox.size_x/2
        d.bbox.center.y = box[0]+d.bbox.size_y/2
        o = ObjectHypothesisWithPose()
        o.score = score
        o.id = clabel
        d.results.append(o)
        message.detections.append(d)
    pub.publish(message)
    rospy.loginfo("published")


si = rospy.Subscriber("webcam_images", Image, rcv_image) #per farlo statico fai "images"
#si = rospy.Subscriber("images", Image, rcv_image) per casa



try:
    rospy.spin()

except KeyboardInterrupt:
    print("Shutting down")
    
