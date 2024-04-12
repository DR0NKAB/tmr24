import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import cv2
import numpy as np
from cv_bridge import CvBridge

latest_message = None

def callback(message):
    global latest_message
    latest_message = message

def line_detect():
    rospy.init_node("cone_detect")
    rospy.loginfo("Nodo de deteccion de conos incializando")

    line_pub = rospy.Publisher("line_detect/lateral_error", Float64, queue_size=10)
    image_pub = rospy.Publisher("cone_detect/detections", Image, queue_size=10)
    rospy.Subscriber("camera/compressed", CompressedImage, callback)
    rospy.sleep(1)

    bridge = CvBridge()
    hsv_min = [20, 50, 0]
    hsv_max = [30, 200, 255]

    rospy.loginfo("Deteccion de linea iniciada")
    while not rospy.is_shutdown():
        if latest_message is not None:
            frame_orig = bridge.compressed_imgmsg_to_cv2(latest_message, desired_encoding="bgr8")

            # convertir de BGR a HSV, aplicar blur para el ruido y aplicar filtro para colores entre hsv_min y hsv_max, para obtener los contornos
            frame = cv2.cvtColor(frame_orig, cv2.COLOR_BGR2HSV)
            frame = cv2.medianBlur(frame, 3)
            frame = cv2.inRange(frame, np.array(hsv_min), np.array(hsv_max))
            
            image_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

            rospy.Rate(30).sleep() # comentado, procesa tan rapido como puede, si no, a la frecuencia especificada
        else:
            rospy.loginfo("Esperando a recibir una imagen")
            rospy.Rate(1).sleep()

if __name__ == "__main__":
    try:
        line_detect()
    except rospy.ROSInterruptException:
        pass
