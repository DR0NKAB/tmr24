import rospy
import cv2
from tmr24.msg import Window
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

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
    rospy.Subscriber("camera/compressed", CompressedImage, callback)
    rospy.sleep(1)

    model = YOLO("ventanas.pt")
    bridge = CvBridge()
    rospy.loginfo("Window detection node started correctly")

    while not rospy.is_shutdown():
        if latest_message is not None:
            class_to_detect = 0
            tipos = [class_to_detect]
            frame = bridge.compressed_imgmsg_to_cv2(latest_message, desired_encoding="bgr8")
            windows = model.predict( frame, classes=tipos, verbose=False, conf=0.40)[0]

            mensaje = Window()

            max_area = 0
            max_box = None

            for box in windows.boxes.xyxy:
                x1, y1, x2, y2 = box.int()
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    max_box = box
                        
            if max_box is not None:
                print("Found a max window with area : ", max_area.numpy())
                print(windows.speed)
                x1, y1, x2, y2 = max_box.int()
                
                mensaje.type = "normal"
                mensaje.x1 = x1
                mensaje.y1 = y1

                mensaje.x2 = x2
                mensaje.y2 = y2

                mensaje.horizontal_error = frame.shape[1] - (x2 - x1) / 2
                mensaje.vertical_error = frame.shape[0] - (y2 - y1) /2
            else:
                print("No window found")
                
            window_pub.publish(mensaje)
            detection_pub.publish( bridge.cv2_to_imgmsg(windows.plot(), encoding="bgr8") )
            #rospy.Rate(10).sleep() # While commented, it will process as fast as it can, otherwise, at the frecuency provided
        else:
            print("Waiting for an image to be received")
            rospy.Rate(1).sleep()

if __name__ == "__main__":
    try:
        window_detect()
    except rospy.ROSInterruptException:
        pass
