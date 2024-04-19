import rospy
import smach
import math
import numpy as np
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialArray

ancho_pantalla = 856
alto_pantalla = 450

class AlignToAruco(smach.State):
    def __init__(self, current_aruco_id, align_yaw = True, aruco_angle = 0):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.aruco_angle = aruco_angle
        self.current_aruco_id = current_aruco_id
        self.current_horizontal_error = None
        self.current_vertical_error = None
        self.current_yaw_error = None
        self.align_yaw = align_yaw

    def callback_transforms(self, message):
        for transform in message.transforms:
            if transform.fiducial_id == self.current_aruco_id:
                # where w is the scalar (real) part and x, y, and z are the vector parts
                x_rotation = transform.transform.rotation.x
                y_rotation = transform.transform.rotation.y
                z_rotation = transform.transform.rotation.z
                w_rotation = transform.transform.rotation.w
                
                # Convert a quaternion into euler angle (yaw)
                t3 = +2.0 * ( w_rotation * z_rotation + x_rotation * y_rotation)
                t4 = +1.0 - 2.0 * (y_rotation * y_rotation + z_rotation * z_rotation)
                yaw_aruco_radian = math.atan2(t3, t4)
                yaw_aruco_degree = yaw_aruco_radian * (180/math.pi)
                self.current_yaw_error = math.sin((self.aruco_angle) * (math.pi/180)) - math.sin(yaw_aruco_radian)
                """if yaw_aruco_degree >= self.aruco_angle:
                    self.current_yaw_error = (-1)*(math.cos((self.aruco_angle - 90) * (math.pi/180)) - math.cos(yaw_aruco_radian) )
                    
                else:
                    self.current_yaw_error = math.cos((self.aruco_angle -90) * (math.pi/180)) - math.cos(yaw_aruco_radian)"""

                rospy.loginfo(f"Estoy recibiendo {self.aruco_angle} grados")
                rospy.loginfo(f"El error es de {self.current_yaw_error}")
                

    def callback_vertices(self, message):
        for vertices in message.fiducials:
            if vertices.fiducial_id == self.current_aruco_id:
                self.latest_vertices = vertices

                x_cords = [vertices.x0, vertices.x1, vertices.x2, vertices.x3]
                center_x = int(np.mean(x_cords))
                self.current_horizontal_error = ancho_pantalla/2 - center_x

                y_cords = [vertices.y0, vertices.y1, vertices.y2, vertices.y3]
                center_y = int(np.mean(y_cords))
                self.current_vertical_error = alto_pantalla/2 - center_y

    def execute(self, userdata):
        rospy.loginfo(f"ALIGN TO ARUCO state executing with aruco id : {self.current_aruco_id} ")
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        aruco_transforms_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback_transforms)
        aruco_vertices_sub = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.callback_vertices)
        rospy.sleep(1)

        camera_angle = -90
        rospy.loginfo(f"Setting camera angle to {camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        sampling_time = 0.1

        rospy.loginfo(f"Entering XY control loop")
        kp_h_v = 0.001
        kd_h_v = 0.001
        last_error_horizontal = 0
        last_error_vertical = 0
        
        counter_h = 0
        counter_v = 0
        tolerance_h_v = 20 #pixeles
        counter_h_limit = 50
        counter_v_limit = 50
        rate = rospy.Rate(1/sampling_time)

        while not rospy.is_shutdown():
            if self.current_vertical_error != None and self.current_horizontal_error != None:

                if abs(self.current_horizontal_error) < tolerance_h_v:
                    rospy.loginfo(f"Counter horizontal : {counter_h}")
                    counter_h = counter_h + 1 

                if abs(self.current_vertical_error) < tolerance_h_v:
                    rospy.loginfo(f"Counter vertical: {counter_v}")
                    counter_v = counter_v + 1 

                if counter_h > counter_h_limit and counter_v > counter_v_limit :
                    rospy.loginfo("Centrado en ejes XY, terminando bucle de control")
                    hover_msg = Twist()
                    movement_pub.publish(hover_msg)
                    rospy.sleep(3)
                    break

                msg = Twist()
                msg.linear.x = kp_h_v * self.current_vertical_error
                msg.linear.y = kp_h_v * self.current_horizontal_error

                msg.linear.x = msg.linear.x + (kd_h_v * (self.current_vertical_error - last_error_vertical) / sampling_time)
                msg.linear.y = msg.linear.y + (kd_h_v * (self.current_horizontal_error - last_error_horizontal) / sampling_time)
                
                movement_pub.publish(msg)

                last_error_horizontal = self.current_horizontal_error
                last_error_vertical = self.current_vertical_error
            rate.sleep()

        kp_yaw = 1
        kd_yaw = 0
        last_error_yaw = 0 
        tolerance_yaw = (math.cos(0) - math.cos(10*(math.pi/180))) #grados
        rospy.loginfo(f"la tolerancia es {tolerance_yaw}")
        #tolerance_yaw = 17
        counter_yaw = 0
        counter_yaw_limit = 50

        if not rospy.is_shutdown():
            if self.align_yaw:
                rospy.loginfo("Llegue al ciclo de Yaw")
                while not rospy.is_shutdown():
                    if self.current_yaw_error != None:
                        
                        if abs(self.current_yaw_error) < abs(tolerance_yaw):
                            rospy.loginfo(f"Counter yaw : {counter_yaw}")
                            counter_yaw = counter_yaw + 1 

                        if counter_yaw > counter_yaw_limit:
                            rospy.loginfo("Centrado en YAW, dejando estado")
                            hover_msg = Twist()
                            movement_pub.publish(hover_msg)
                            rospy.sleep(3)
                            return "succeeded"

                        msg = Twist()
                        msg.angular.z = kp_yaw * self.current_yaw_error
                        msg.angular.z = msg.angular.z + (kd_yaw * (self.current_yaw_error - last_error_yaw)/sampling_time)

                        movement_pub.publish(msg)
                        last_error_yaw = self.current_yaw_error

                    rate.sleep()
            return "succeeded"

        return "failed"