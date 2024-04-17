import rospy
import cv2
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

    rospy.init_node("window_detect")
    rospy.loginfo("Window detection node initializing")

    window_pub = rospy.Publisher("window_detect/windows", Window, queue_size=10)
    detection_pub = rospy.Publisher("window_detect/detections", Image, queue_size=10)
    rospy.Subscriber("bebop/image_raw/compressed", CompressedImage, callback)
    rospy.sleep(1)

    model = YOLO("ventanas.pt")
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
            hsv_min1 = [0, 86, 39]
            hsv_max1 = [8, 250, 255]
            mask1 = cv2.inRange(imgHSV, np.array(hsv_min1), np.array(hsv_max1))

            # rojo
            hsv_min2 = [150, 74, 60]
            hsv_max2 = [200, 255, 255]
            mask2 = cv2.inRange(imgHSV, np.array(hsv_min2), np.array(hsv_max2))

            frame_with_filter = np.zeros_like(imgHSV)
            frame_with_filter[(mask1 > 0) | (mask2 > 0)] = [0, 0, 255]

            #####################################

            predictions = model.predict( frame_with_filter, classes=tipos, verbose=False, conf=0.40)[0]

            mensaje = Window()
            max_area = 0
            max_box = None

            for box in predictions.boxes.xyxy:
                x1, y1, x2, y2 = box.int()
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    max_box = box
                        
            if max_box is not None:
                rospy.loginfo(f"Found a max window with area : {max_area.numpy()}")
                rospy.loginfo(predictions.speed)
                
                x1, y1, x2, y2 = max_box.int()
                rospy.loginfo(f" ancho = {x2 - x1} ")
                rospy.loginfo(f"alto = {y2 - y1} ")

                mensaje.type = "normal"
                mensaje.area = max_area.numpy()
                mensaje.x1 = x1
                mensaje.y1 = y1

                mensaje.x2 = x2
                mensaje.y2 = y2

                mensaje.horizontal_error = frame.shape[1] / 2 - (x2 + x1) / 2
                mensaje.vertical_error = frame.shape[0] / 2 - y2 
                window_pub.publish(mensaje)
            else:
                rospy.loginfo("No window found")
            
            window_pub.publish(mensaje)
            detection_pub.publish( bridge.cv2_to_imgmsg(predictions.plot(), encoding="bgr8") )
            # rate.sleep() # While commented, it will process as fast as it can, otherwise, at the frecuency provided
        else:
            rospy.loginfo("Waiting for an image to be received")
            rospy.sleep(1)

if __name__ == "__main__":
    try:
        window_detect()
    except rospy.ROSInterruptException:
        pass
