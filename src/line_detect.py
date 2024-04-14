import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import cv2
import numpy as np
from cv_bridge import CvBridge

# Constantes para definir los valores de color según el número de filtro
COLOR_SET_1 = np.array([92, 59, 211])   # Rojo
COLOR_SET_2 = np.array([156, 225, 255])   # Amarillo
COLOR_SET_3 = np.array([117, 81, 89])   # Azul

SIZE_FILTER_1 = 40  # Tamaño del filtro para COLOR_SET_1
SIZE_FILTER_2 = 57  # Tamaño del filtro para COLOR_SET_2
SIZE_FILTER_3 = 20  # Tamaño del filtro para COLOR_SET_3

latest_message = None
bridge = CvBridge()

def callback(message):
    global latest_message
    latest_message = message

def create_binary_image(frame, selected_color, size_filter):
    lower_color = selected_color - np.array([size_filter, size_filter, size_filter])
    upper_color = selected_color + np.array([size_filter, size_filter, size_filter])
    mask = cv2.inRange(frame, lower_color, upper_color)
    return mask

def calculate_error(contours, frame_width):
    if not contours:
        return 0

    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        center_x = int(M["m10"] / M["m00"])
    else:
        return 0

    center_row = frame_width // 2
    error = center_x - center_row
    return error

def video_feed(frame, num_sections, select_filter, image_pub):
    if select_filter == 1:
        selected_color = COLOR_SET_1
        size_filter = SIZE_FILTER_1
    elif select_filter == 2:
        selected_color = COLOR_SET_2
        size_filter = SIZE_FILTER_2
    elif select_filter == 3:
        selected_color = COLOR_SET_3
        size_filter = SIZE_FILTER_3
    else:
        return  # Salir si el tipo de filtro no es válido

    r, g, b = selected_color[2], selected_color[1], selected_color[0]
    binary_image = create_binary_image(frame, np.array([b, g, r]), size_filter)

    threshold_value = 50  # Umbral fijo (puedes ajustar según sea necesario)
    kernel_size = 3  # Tamaño de kernel fijo (puedes ajustar según sea necesario)

    blurred = cv2.GaussianBlur(binary_image, (kernel_size, kernel_size), 0)
    _, thresh = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)

    width = frame.shape[1]
    half_width = width // 2
    error_sum = 0
    section_height = frame.shape[0] // num_sections

    for i in range(num_sections):
        section_start = i * section_height
        section_end = (i + 1) * section_height
        section = thresh[section_start:section_end, :]
        contours, _ = cv2.findContours(section, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calcular el error para la sección actual
        error = calculate_error(contours, width)
        error_sum += error

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            largest_contour[:, :, 1] += section_start
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.line(frame, (center_x, center_y), (half_width, center_y), (0, 255, 255), 1)
    
    image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    image_pub.publish(image_msg)
    #line_pub.publish(Float64(error_avg))
    #cv2.imshow('Frame', frame)
    error_avg = error_sum / num_sections
    print(f"El error es: {error_avg}")

def line_detect():
    rospy.init_node("line_detect")
    rospy.loginfo("Nodo de deteccion de linea incializando")

    line_pub = rospy.Publisher("line_detect/lateral_error", Float64, queue_size=10)
    image_pub = rospy.Publisher("line_detect_detect/detections", Image, queue_size=10)
    rospy.Subscriber("bebop/image_raw/compressed", CompressedImage, callback)
    rospy.loginfo("Esperando a recibir una imagen")
    rospy.sleep(1)
    bridge = CvBridge()
    rate = rospy.Rate(30)

    num_sections = 20
    select_filter = 2  # Seleccionar el filtro amarillo (2)
    while not rospy.is_shutdown():
        if latest_message is not None:
            frame_orig = bridge.compressed_imgmsg_to_cv2(latest_message, desired_encoding="bgr8")
            video_feed(frame_orig, num_sections, select_filter,image_pub)
            rate.sleep() # comentado, procesa tan rapido como puede, si no, a la frecuencia especificada
        else:
            rospy.loginfo("Esperando a recibir una imagen")
            rospy.sleep(1)
        
if __name__ == "__main__":
    try:
        line_detect()
    except rospy.ROSInterruptException:
        pass
