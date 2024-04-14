import rospy
import smach
import math
import numpy as np
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransform
from fiducial_msgs.msg import Fiducial

class GoToAruco(smach.State):
    def __init__(self, current_aruco_id, next_aruco_id, camera_angle):
        smach.State.__init__(self, outcomes=["succeeded", "skipped","failed"])
        self.camera_angle = camera_angle
        self.current_aruco_id = current_aruco_id
        self.next_aruco_id = next_aruco_id
        self.latest_transform = None
        self.latest_vertices = None
        self.fiducial_id_found = None
        self.yaw_error = 0

    def callback(self, message):
        if self.fiducial_id_found is not None:
            if message.fiducial_id == self.fiducial_id_found:
                self.latest_transform = message
        else:
            if message.fiducial_id == self.current_aruco_id or message.fiducial_id == self.next_aruco_id:
                rospy.loginfo(f"Encontre un aruco\n\n\n\n\n\n\n\n\n\n, con id : {message.fiducial_id}")
                self.fiducial_id_found = message.fiducial_id

    def callback2(self, message):
        if self.fiducial_id_found is not None:
            if message.fiducial_id == self.fiducial_id_found:
                self.latest_vertices = message


    def execute(self, userdata):
        rospy.loginfo(f"GOTOARUCO state executing with aruco id : {self.current_aruco_id} skipping if aruco id : {self.next_aruco_id} is found")
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        aruco_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransform, self.callback)
        aruco_sub2 = rospy.Subscriber("/fiducial_vertices", Fiducial, self.callback2)
        
        rospy.sleep(1)

        rospy.loginfo(f"Setting camera angle to {self.camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = self.camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        rospy.loginfo(f"Searching for aruco {self.current_aruco_id} or {self.next_aruco_id}")
        rate = rospy.Rate(30)
        distance = 0
        Kp = 1.0
        Kd = 0.2
        tolerance = 20
        zero_error_counter = 0

        while not rospy.is_shutdown():
            if self.latest_transform is not None:
                if self.latest_vertices is not None:
                    x_cords = [self.latest_vertices.x0, self.latest_vertices.x1, self.latest_vertices.x2, self.latest_vertices.x3]
                    y_cords = [self.latest_vertices.y0, self.latest_vertices.y1, self.latest_vertices.y2, self.latest_vertices.y3]
                    center_x = int(np.mean(x_cords))
                    center_y = int(np.mean(y_cords))
                    yaw_error = 856/2 - center_x

                    if abs(yaw_error) < tolerance:
                        zero_error_counter = zero_error_counter + 1
                    else:
                        zero_error_counter = 0
                        movement_msg = Twist()
                        movement_msg.angular.z = Kp*yaw_error

                    if zero_error_counter >= 500:
                        
                        rospy.loginfo("ESTOY CENTRADO")
                        """"
                        aruco_sub.unregister()

                        rospy.loginfo(f"Found aruco {self.fiducial_transform.fiducial_id}")
                        
                        rospy.loginfo("Sending command to go to Aruco")
                        time_to_aruco = 5
                        gain_vel_to_twist = 0.1
                        #forward_dist = self.fiducial_transform.transform.translation.z
                        #left_dist = self.fiducial_transform.transform.translation.x
                        movement_msg = Twist()
                        movement_msg.linear.x = (distance / time_to_aruco) * gain_vel_to_twist 
                        #movement_msg.linear.y = (left_dist / time_to_aruco) * gain_vel_to_twist 
                        movement_pub.publish(movement_msg)
                        rospy.loginfo(f"Command was {movement_msg}")

                        rospy.loginfo("Waiting for command to be completed")
                        rospy.sleep(time_to_aruco)

                        if self.fiducial_transform.fiducial_id == self.current_aruco_id:
                            return "succeeded"
                        else :
                            return "skipped" """
                        
                rate.sleep()
            else: 
                rospy.loginfo("No encontre un aruco, movere la camara")
                movement_msg = Twist()
                movement_msg.angular.z = 0.05
                movement_pub.publish(movement_msg)
                rospy.sleep(1)
        return "failed"