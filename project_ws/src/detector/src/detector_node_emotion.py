#!/usr/bin/env python3
import os
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from detector import Detector
import ros_numpy # pip3 install git+https://github.com/eric-wieser/ros_numpy
import cv2
import tensorflow as tf

DET_PATH=os.path.join(os.path.dirname(__file__),'efficientdet_d1_coco17_tpu-32')
mydetector = Detector(DET_PATH)

rospy.init_node('detector_node_emotion')
pub = rospy.Publisher('detection', Detection2DArray, queue_size=2)


faceProto = "/home/sirc/Scrivania/cogrob/cogrob_ws/src/detector/src/EmotionModels/opencv_face_detector.pbtxt"
faceModel = "/home/sirc/Scrivania/cogrob/cogrob_ws/src/detector/src/EmotionModels/opencv_face_detector_uint8.pb"
mydetector = cv2.dnn.readNet(faceModel, faceProto) #faceNet

def getFaceBox(msg):
    conf_threshold=0.8
    image = ros_numpy.numpify(msg)
    message = Detection2DArray()

    frameOpencvDnn = image.copy()
    frameHeight = frameOpencvDnn.shape[0]
    frameWidth = frameOpencvDnn.shape[1]
    
    #swapRB =True
    # flag which indicates that swap first and last channels in 3-channel image is necessary.
    #crop = False
    # flag which indicates whether image will be cropped after resize or not
    # If crop is false, direct resize without cropping and preserving aspect ratio is performed
    blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], True, False)

    mydetector.setInput(blob)
    detections = mydetector.forward()
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold and detections[0, 0, i, 5]<1 and detections[0, 0, i, 6]<1:
            d = Detection2D()
            x1 = int(detections[0, 0, i, 3] * frameWidth) #center x
            y1 = int(detections[0, 0, i, 4] * frameHeight) #center y
            x2 = int(detections[0, 0, i, 5] * frameWidth) #size x
            y2 = int(detections[0, 0, i, 6] * frameHeight) #size y, UTILIZZANDO QUESTI FUNZIONA!

            d.bbox.size_x = x1
            d.bbox.size_y = y1
            d.bbox.center.x = x2
            d.bbox.center.y = y2
           
            message.detections.append(d)
    pub.publish(message)
    #cv2.imshow('Image', frameOpencvDnn)
    #cv2.waitKey(100)
    rospy.loginfo("published")




si = rospy.Subscriber("image", Image, getFaceBox)



try:
    rospy.spin()

except KeyboardInterrupt:
    print("Shutting down")
    