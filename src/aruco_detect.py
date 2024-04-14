import rospy
import smach
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from tmr24.msg import Aruco

import cv2
import numpy as np
from cv_bridge import CvBridge

latest_message = None


def callback(message):
    global latest_message
    latest_message = message

# Fórmula para calcular el ángulo según el pixel en la coordenada y
def calcular_angulo(y_pixel):
    x = y_pixel
    return 85 - 0.0582642 * x + 0.00153083 * x**2 - 0.0000191457 * x**3 + \
    1.17508e-7 * x**4 - 3.84105e-10 * x**5 + 6.37131e-13 * x**6 - 4.25932e-16 * x**7

def aruco_detect():
    rospy.init_node("aruco_detect")
    rospy.loginfo("Ndo de deteccion de arucos incializando")

    aruco_pub = rospy.Publisher("aruco_detect/arucos", Aruco, queue_size=10)
    detection_pub = rospy.Publisher("aruco_detect/detections", Image, queue_size=10)
    rospy.Subscriber("/bebop/image_raw", Image, callback)
    rospy.sleep(1)

    bridge = CvBridge()
    rospy.loginfo("Deteccion de arucos iniciada")
        
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if latest_message is not None:
            frame_orig = bridge.compressed_imgmsg_to_cv2(latest_message, desired_encoding="bgr8")

            ############################################################################

            # Altura del dron conocida
            altura_dron = 1.5  # Supongamos que la altura es de 1.5 metros
                
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
            parameters = cv2.aruco.DetectorParameters_create()

            midpoint_x=frame_orig.shape[0]/2
            midpoint_y=frame_orig.shape[1]/2

            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame_orig, aruco_dict, parameters=parameters)

            if ids is not None:
                for i in range(len(ids)):
                    id = ids[i][0]  # ID del marcador actual
                    corner_points = corners[i][0]  # Coordenadas de las esquinas del marcador actual
                    corner_points_int = corner_points.astype(int)
                    #print(f"Marcador ID {id}: Coordenadas de esquinas {corner_points_int}")

                    # Calcular el centro del marcador como el promedio de las coordenadas de las esquinas
                    center_x = int(np.mean(corner_points[:, 0]))
                    center_y = int(np.mean(corner_points[:, 1]))
                    #print(f"Centro del marcador ID {id}: ({center_x}, {center_y})")

                    # Calcular el ángulo aproximado basado en el pixel y
                    angulo = calcular_angulo(center_y)

                    # Calcular la distancia usando la altura del dron y el ángulo
                    distancia = altura_dron * np.tan(np.radians(angulo))
                    #print(f"Ángulo aproximado: {angulo} grados")
                    #print(f"Distancia estimada: {distancia} metros")
                    data_message=Aruco()
                    data_message.id = id
                    data_message.x_1 = corner_points[0][0]
                    data_message.x_2 = corner_points[1][0]
                    data_message.x_3 = corner_points[2][0]
                    data_message.x_4 = corner_points[3][0]
                    data_message.y_1 = corner_points[0][1]
                    data_message.y_2 = corner_points[1][1]
                    data_message.y_3 = corner_points[2][1]
                    data_message.y_4 = corner_points[3][1]
                    data_message.error_yaw = midpoint_x-center_x
                    data_message.distance = distancia
                    aruco_pub.publish(data_message)
            rate.sleep()


            frame_markers = cv2.aruco.drawDetectedMarkers(frame_orig, corners, ids)

                # Mostrar la imagen con los marcadores
                # cv2.imshow('Detección ArUco', frame_markers)

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

        ###109.6mm