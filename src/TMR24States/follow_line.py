import rospy
import smach
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from fiducial_msgs.msg import FiducialTransformArray

aruco_id_to_end = 900

class FollowLine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.current_error = 0
        self.current_red_area = 0
        self.aruco_found = False

    def line_error_callback(self, message):
        self.current_error = message.data

    def red_area_callback(self, message):
        self.current_red_area = message.data

    def aruco_callback(self, message):
        global aruco_id_to_end
        for transform in message.transforms:
            if transform.fiducial_id == aruco_id_to_end:
                self.aruco_found = True

    def execute(self, userdata):
        global aruco_id_to_end
        rospy.loginfo("FOLLOWLINE state executing")
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        line_error_sub = rospy.Subscriber("/line_detect/lateral_error", Float64, self.line_error_callback)
        red_area_sub = rospy.Subscriber("/line_detect/red_area", Float64, self.red_area_callback)
        aruco_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.aruco_callback)
        rospy.sleep(1)

        camera_angle = -60
        rospy.loginfo(f"Setting camera angle to {camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was : {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        rospy.loginfo("Entering control loop")
        tiempo_muestreo = 0.1
        kp = 0.00015
        kd = 0.001
        rate = rospy.Rate(1/tiempo_muestreo)
        error_anterior = 0
        while not rospy.is_shutdown():

            if self.current_red_area > 5000:
                rospy.loginfo("Possible window detected, terminating state")
                rospy.loginfo("Hovering for 3 seconds and moving to next state")
                msg = Twist()
                msg.linear.x = -0.02
                movement_pub.publish(msg)
                rospy.sleep(3)
                movement_pub.publish(Twist())
                rospy.sleep(3)
                return "succeeded"
            elif self.aruco_found : 
                rospy.loginfo(f"Aruco with id {aruco_id_to_end} found, leaving state.")
                rospy.loginfo("Hovering for 3 seconds and moving to next state")
                movement_pub.publish(Twist())
                rospy.sleep(3)
                return "succeeded"
            else:
                msg = Twist()
                msg.linear.x = 0.02
                msg.linear.y = kp * self.current_error

                msg.linear.y = msg.linear.y + ( kd * (self.current_error - error_anterior) / tiempo_muestreo ) 

                movement_pub.publish(msg)
            
            error_anterior = self.current_error
            rate.sleep()

        return "failed"