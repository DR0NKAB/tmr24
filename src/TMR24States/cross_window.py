import rospy
import smach
import math
from geometry_msgs.msg import Twist
from tmr24.msg import Window

opc = 1 # opc 1 normal, 2 obstaculo

class CrossWindow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.current_h_error = 0
        self.current_v_error = 0
        self.current_area = 0

    def callback(self, message):
        self.current_h_error = message.horizontal_error
        self.current_v_error = message.vertical_error
        self.current_area = message.area

    def execute(self, userdata):
        rospy.loginfo("CROSSWINDOW state executing")
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        window_error_sub = rospy.Subscriber("/window_detect/windows", Window, self.callback)
        rospy.sleep(1)

        camera_angle = -1
        rospy.loginfo(f"Setting camera angle to {camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was : {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        rospy.loginfo("Entering control loop")
        tiempo_muestreo = 0.5
        kp_h = 0.0001
        kp_v = 0.0003
        # kp_fw = 0.0000002
        kd_h = 0.0000
        kd_v = 0.0000
        rate = rospy.Rate(1/tiempo_muestreo)
        zero_error_h_counter = 0
        zero_error_v_counter = 0
        minimal_counter = 80
        h_tolerance = 50
        v_tolerance = 50
        max_area = 1000
        back_h_error = 0
        back_v_error = 0
        while not rospy.is_shutdown():
            
            if abs(self.current_h_error) < h_tolerance:
                zero_error_h_counter = zero_error_h_counter + 1
            else:
                zero_error_h_counter = 0

            if abs(self.current_v_error) < v_tolerance:
                zero_error_v_counter = zero_error_v_counter + 1
            else:
                zero_error_v_counter = 0
                
            rospy.loginfo(f"contador h: {zero_error_h_counter}")
            rospy.loginfo(f"contador v: {zero_error_v_counter}")

            # opc 2
            if opc == 2:
                if zero_error_h_counter >= minimal_counter and zero_error_v_counter >= minimal_counter and self.current_area > max_area:
                    if True:
                        rospy.loginfo("Yo cruzaria ahora")
                        
                    msg.linear.x = 0.2
                    
                    movement_pub.publish(msg)
                    rospy.sleep(5)
                    movement_pub.publish(Twist())
                    return "succeeded"
            
            # opc 1
            if opc == 1:
                if zero_error_h_counter >= minimal_counter and  zero_error_v_counter >= minimal_counter:
                    if True:
                        rospy.loginfo("Centrado")
                    
                    msg.linear.x = 0.2
                    movement_pub.publish(msg)
                    rospy.sleep(5)
                    movement_pub.publish(Twist())
                    return "succeeded"

            msg = Twist()
            msg.linear.y = kp_h * self.current_h_error + kd_h * (self.current_h_error - back_h_error) / tiempo_muestreo
            msg.linear.z = kp_v * self.current_v_error + kd_v * (self.current_v_error - back_v_error) / tiempo_muestreo
            movement_pub.publish(msg)

            back_h_error = self.current_h_error
            back_v_error = self.current_v_error
            rate.sleep()

        return "failed"