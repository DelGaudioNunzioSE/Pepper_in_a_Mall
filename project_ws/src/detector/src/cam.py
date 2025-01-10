#!/usr/bin/env python3
#file di prova per vedere se il detector prende immagini dalla webcam

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    # Inizializza il nodo ROS
    rospy.init_node('webcam_image_publisher')
    
    # Crea il publisher per il topic "webcam_images"
    image_pub = rospy.Publisher('webcam_images', Image, queue_size=10)
    
    # Crea un oggetto CvBridge per convertire le immagini OpenCV in messaggi ROS
    bridge = CvBridge()
    
    # Apri la webcam (di solito, 0 è la webcam predefinita)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Errore nell'aprire la webcam.")
        return
    
    rospy.loginfo("Acquisizione immagini dalla webcam...")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Immagine non ricevuta dalla webcam.")
            break
        
        # Converti l'immagine OpenCV in un messaggio ROS Image
        try:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_pub.publish(ros_image)
            rospy.loginfo("Immagine inviata su 'webcam_images'")
        except Exception as e:
            rospy.logerr(f"Errore nella conversione dell'immagine: {e}")
        
        # Aggiungi un controllo per limitare la frequenza di pubblicazione
        rospy.sleep(0.1)  # Pubblica circa ogni 0.1 secondi (10Hz)

    # Rilascia la webcam quando il nodo viene interrotto
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

