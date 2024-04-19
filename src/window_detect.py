import rospy
import cv2
import math
import numpy as np
from tmr24.msg import Window
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

opc = 1 # opc 1 normal, 2 obstaculo
latest_message = None

def callback(message):
    global latest_message
    latest_message = message

def window_detect():
    global latest_message
    global opc

    last_center = None

    rospy.init_node("window_detect")
    rospy.loginfo("Window detection node initializing")

    window_pub = rospy.Publisher("window_detect/windows", Window, queue_size=10)
    detection_pub = rospy.Publisher("window_detect/detections", Image, queue_size=10)
    rospy.Subscriber("bebop/image_raw/compressed", CompressedImage, callback)
    rospy.sleep(1)

    model = YOLO("yolov8n.pt")
    bridge = CvBridge()
    rospy.loginfo("Window detection node started correctly")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_message is not None:
            class_to_detect = 0
            tipos = [class_to_detect]
            frame = bridge.compressed_imgmsg_to_cv2(latest_message, desired_encoding="bgr8")

            ############ Filtros ################

            imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # naranja
            hsv_min1 = [0, 106, 59]
            hsv_max1 = [15, 255, 255]
            mask1 = cv2.inRange(imgHSV, np.array(hsv_min1), np.array(hsv_max1))

            # rojo
            hsv_min2 = [150, 120, 26]
            hsv_max2 = [200, 255, 255]
            mask2 = cv2.inRange(imgHSV, np.array(hsv_min2), np.array(hsv_max2))

            frame_with_filter = np.zeros_like(imgHSV)
            frame_with_filter[(mask1 > 0) | (mask2 > 0)] = [0, 0, 255]

            #####################################

            predictions = model.predict( frame, classes=tipos, verbose=False, conf=0.40)[0]
            frame_to_publish = predictions.plot()

            mensaje = None
            max_area = 0
            max_box = None
            min_left_distance = 0
            if opc == 1:
                min_left_distance = 1000000
            dif_x = 50
            dif_y = 20

            for box in predictions.boxes.xyxy:
                x1, y1, x2, y2 = box.int()
                area = (x2 - x1) * (y2 - y1)
                left_distance = (x2 + x1) / 2 

                if opc == 1:
                    if left_distance < min_left_distance:
                        left_distance = min_left_distance
                        max_area = area
                        max_box = box

                if opc == 2:
                    if left_distance > min_left_distance:
                        left_distance = min_left_distance
                        max_area = area
                        max_box = box

                
                        
            if max_box is not None:
                mensaje = Window()
                rospy.loginfo(f"Found a max window with area {max_area.numpy()}")
                rospy.loginfo(predictions.speed)

                x1, y1, x2, y2 = max_box.int()
                cv2.circle(frame_to_publish, (int((x1+x2)/2), int((y1+y2)/2)), 5, (0, 255, 255), 5) # esto dibuja un circulo en el centro del contorno
                
                x1, y1, x2, y2 = max_box.int()

                if opc==1:
                    mensaje.type = "normal"
                    mensaje.x1 = x1
                    mensaje.y1 = y1

                    mensaje.x2 = x2
                    mensaje.y2 = y2

                    mensaje.horizontal_error = frame.shape[1] / 2 - (x2 + x1) / 2
                    mensaje.vertical_error = frame.shape[0] / 2 - (y2 + y1) / 2
                    window_pub.publish(mensaje)

                if opc==2:
                    mensaje.type = "obstaculo" 
                    centro_x = (x2 + x1) / 2
                    centro_y = (y2 + y1) / 2
                    new_coords = (centro_x - dif_x, centro_y + dif_y, centro_x + dif_x, int(y2) - dif_y - 30)

                    #mensaje.area = (new_coords[2] - new_coords[0]) * (new_coords[3] - new_coords[1])
                    mensaje.x1 = new_coords[0]
                    mensaje.x2 = new_coords[2]
                    mensaje.y1 = new_coords[1]
                    mensaje.y2 = new_coords[3]

                    new_x = new_coords[2] - new_coords[0]
                    new_y = new_coords[3] - new_coords[1]

                    analisis_pix = frame[new_coords[1]:new_coords[3], new_coords[0]:new_coords[2]]

                    ###verificar si new_coords está dentro de los límites de la imagen

                    if new_coords[0] >= 0 and new_coords[1] >= 0 and new_coords[2] <= frame.shape[0] and new_coords[3] <= frame.shape[1] and analisis_pix.size > 0:

                        imgHSV_blanco = cv2.cvtColor(analisis_pix, cv2.COLOR_BGR2HSV)
                        mask1 = cv2.inRange(imgHSV_blanco, np.array(hsv_min1), np.array(hsv_max1))
                        mask2 = cv2.inRange(imgHSV_blanco, np.array(hsv_min2), np.array(hsv_max2))

                        img_analisis = np.zeros_like(analisis_pix)
                        img_analisis[(mask1 > 0) | (mask2 > 0)] = [255]  # cambiando a blanco

                        img_analisis = cv2.cvtColor(img_analisis, cv2.COLOR_BGR2GRAY)

                        # pixeles rojos conteo
                        pixel_count = cv2.countNonZero(img_analisis)

                    mensaje.area = pixel_count
                    mensaje.horizontal_error = frame.shape[1] / 2 - new_x
                    mensaje.vertical_error = frame.shape[0] / 2 - new_y
                    window_pub.publish(mensaje)
                    # start - linea - ventana 
                    
                
            else:
                rospy.loginfo("No window found")
            
            if mensaje is not None:
                window_pub.publish(mensaje)
            detection_pub.publish( bridge.cv2_to_imgmsg(frame_to_publish, encoding="bgr8") )
            # rate.sleep() # While commented, it will process as fast as it can, otherwise, at the frecuency provided
        else:
            rospy.loginfo("Waiting for an image to be received")
            rospy.sleep(1)

if __name__ == "__main__":
    try:
        window_detect()
    except rospy.ROSInterruptException:
        pass