import rospy
import smach
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialArray

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

    def callback_transforms(self, message):
        for transform in message.transforms:
            if self.fiducial_id_found is not None:
                if transform.fiducial_id == self.fiducial_id_found:
                    self.latest_transform = transform
            else:
                if transform.fiducial_id == self.current_aruco_id or transform.fiducial_id == self.next_aruco_id:
                    rospy.loginfo(f"Encontre un aruco con id : {transform.fiducial_id}")
                    self.fiducial_id_found = transform.fiducial_id

    def callback_vertices(self, message):
        if self.fiducial_id_found is not None:
            for vertices in message.fiducials:
                if vertices.fiducial_id == self.fiducial_id_found:
                    self.latest_vertices = vertices
                    x_cords = [vertices.x0, vertices.x1, vertices.x2, vertices.x3]
                    center_x = int(np.mean(x_cords))
                    self.yaw_error = 856/2 - center_x

    def execute(self, userdata):
        rospy.loginfo(f"GOTOARUCO state executing with aruco id : {self.current_aruco_id} skipping if aruco id : {self.next_aruco_id} is found")
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        aruco_transforms_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback_transforms)
        aruco_vertices_sub = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.callback_vertices)
        
        rospy.sleep(1)

        rospy.loginfo(f"Setting camera angle to {self.camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = self.camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        rospy.loginfo(f"Searching for aruco {self.current_aruco_id} or {self.next_aruco_id}")
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.latest_transform is not None:

                kp = 0.0002
                tolerance = 80
                zero_error_counter = 0
                zero_error_limit = 40
                sampling_time = 0.1
                rospy.loginfo("Entrando al control de yaw")
                yaw_rate = rospy.Rate(1/sampling_time)
                while not rospy.is_shutdown():
                    if abs(self.yaw_error) < tolerance:
                        zero_error_counter = zero_error_counter + 1
                    else:
                        zero_error_counter = 0

                    rospy.loginfo(f"zero conter is : {zero_error_counter}")

                    movement_msg = Twist()
                    movement_msg.angular.z = kp*self.yaw_error

                    movement_pub.publish(movement_msg)

                    if zero_error_counter >= zero_error_limit:
                        rospy.loginfo("ESTOY CENTRADO")
                        break
                    yaw_rate.sleep()
                    
                blind_frame = 0
                blind_limit = 10
                self.latest_transform = None
                movement_msg=Twist()
                movement_msg.linear.x=0.02
                movement_pub.publish(movement_msg)
                while not rospy.is_shutdown():
                    if self.latest_transform is not None:
                        blind_frame = 0
                        self.latest_transform = None
                    else:
                        blind_frame = blind_frame + 1

                    if blind_frame > blind_limit:
                        movement_pub.publish(Twist())
                        rospy.sleep(3)
                        if self.fiducial_id_found == self.current_aruco_id:
                            return "succeeded"    
                        else:
                            return "skipped"
                    yaw_rate.sleep()
            else: 
                rospy.loginfo("No encontre un aruco, movere la camara")
                movement_msg = Twist()
                movement_msg.angular.z = -0.1
                movement_pub.publish(movement_msg)
                
            rate.sleep()

        return "failed"