import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from tmr24.msg import Cone

import cv2
import numpy as np
from cv_bridge import CvBridge

latest_message = None

def callback(message):
    global latest_message
    latest_message = message

def cone_detect():
    rospy.init_node("cone_detect")
    rospy.loginfo("Nodo de deteccion de conos incializando")

    cone_pub = rospy.Publisher("cone_detect/cones", Cone, queue_size=10)
    image_pub = rospy.Publisher("cone_detect/detections", Image, queue_size=10)
    rospy.Subscriber("bebop/image_raw/compressed", CompressedImage, callback)
    rospy.sleep(1)

    bridge = CvBridge()
    hsv_min = [20, 50, 0]
    hsv_max = [30, 200, 255]

    rospy.loginfo("Deteccion de conos iniciada")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if latest_message is not None:
            frame_orig = bridge.compressed_imgmsg_to_cv2(latest_message, desired_encoding="bgr8")

            # convertir de BGR a HSV, aplicar blur para el ruido y aplicar filtro para colores entre hsv_min y hsv_max, para obtener los contornos
            frame = cv2.cvtColor(frame_orig, cv2.COLOR_BGR2HSV)
            frame = cv2.medianBlur(frame, 3)
            frame = cv2.inRange(frame, np.array(hsv_min), np.array(hsv_max))
            unfiltered_contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # para cada contorno solo dejar pasar aquellos con area mayor a la establecida en pixeles
            min_contour_area = 0
            contours = [c for c in unfiltered_contours if cv2.contourArea(c) > min_contour_area]

            # solo si se encontro algun contorno
            if len(contours) > 0:

                # obtener el contorno con area mayor y calcular sus momentos
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)

                # evitar division entre cero con este if
                if not M["m00"] == 0:
                    cX = int(M["m10"] / M["m00"])  # Esto encuentra el centro horizontal del contorno.
                    cY = int(M["m01"] / M["m00"])  # Esto encuentra el centro vertical del contorno.

                    cv2.circle(frame_orig, (cX, cY), 5, (0, 255, 255), 5) # esto dibuja un circulo en el centro del contorno
                    frame_orig = cv2.drawContours(frame_orig, [c], -1, (255,255,0), 3) # esto dibuja el contorno en el frame

                    rospy.loginfo(f"Las coordenadas del centro del cono son {cX}, {cY} con area : {cv2.contourArea(c)}")
                    msg = Cone()
                    msg.horizontal_error = frame.shape[1] // 2 - cX
                    msg.vertical_error = frame.shape[0] // 2 - cY
                    cone_pub.publish(msg)
                    image_pub.publish(bridge.cv2_to_imgmsg(frame_orig, encoding="bgr8"))
                else:
                    rospy.loginfo("Contorno no valido")
            else:
                rospy.loginfo("No se encontro el cono")
            rate.sleep() # comentado, procesa tan rapido como puede, si no, a la frecuencia especificada
        else:
            rospy.loginfo("Esperando a recibir una imagen")
            rospy.sleep(1)

if __name__ == "__main__":
    try:
        cone_detect()
    except rospy.ROSInterruptException:
        pass
