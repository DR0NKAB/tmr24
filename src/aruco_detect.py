import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialTransform

import cv2
import numpy as np
from cv_bridge import CvBridge

latest_message = None

def callback(message):
    global latest_message
    latest_message = message

def aruco_detect():
    rospy.init_node("aruco_detect")
    rospy.loginfo("Nodo de deteccion de arucos incializando")

    aruco_pub = rospy.Publisher("fiducial_transforms", FiducialTransform, queue_size=10)
    detection_pub = rospy.Publisher("fiducial_images", Image, queue_size=10)
    rospy.Subscriber("camera/compressed", CompressedImage, callback)
    rospy.sleep(1)

    bridge = CvBridge()
    rospy.loginfo("Deteccion de arucos iniciada")
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if latest_message is not None:
            frame_orig = bridge.compressed_imgmsg_to_cv2(latest_message, desired_encoding="bgr8")

            ############################################################################

            ############################################################################
            
            rate.sleep() # comentado, procesa tan rapido como puede, si no, a la frecuencia especificada
        else:
            rospy.loginfo("Esperando a recibir una imagen")
            rospy.sleep(1)

if __name__ == "__main__":
    try:
        aruco_detect()
    except rospy.ROSInterruptException:
        pass
