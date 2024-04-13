import rospy
import smach
import math
from geometry_msgs.msg import Twist
from tmr24.msg import Window

class CrossWindow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.current_h_error = 0
        self.current_v_error = 0

    def callback(self, message):
        self.current_h_error = message.horizontal_error
        self.current_v_error = message.vertical_error

    def execute(self, userdata):
        rospy.loginfo("CROSSWINDOW state executing")
        movement_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        window_error_sub = rospy.Subscriber("/window_detect/detections", Window, self.callback)
        rospy.sleep(1)

        camera_angle = 0
        rospy.loginfo(f"Setting camera angle to {camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was : {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        rospy.loginfo("Entering control loop")
        tiempo_muestreo = 0.1
        kp_h = 0.01
        kp_v = 0.01
        rate = rospy.Rate(1/tiempo_muestreo)
        while not rospy.is_shutdown():
            if math.isnan(self.current_h_error) and math.isnan(self.current_v_error) :
                rospy.loginfo("Crossing window !!!")
                movement_msg = Twist()
                movement_msg.linear.x = 0.2
                movement_pub.publish(movement_msg)
                rospy.loginfo(f"Command was : {movement_msg}")
                rospy.loginfo("Waiting for window to be crossed")
                rospy.sleep(2)
                return "succeeded"
            else:
                rospy.loginfo("Sending control command")
                movement_msg = Twist()

                if not math.isnan(self.current_h_error):
                    movement_msg.linear.y = kp_h * self.current_h_error

                if not math.isnan(self.current_v_error):
                    movement_msg.linear.z = kp_v * self.current_v_error

                movement_pub.publish(movement_msg)
                rospy.loginfo(f"Command was : {movement_msg}")
            rate.sleep()

        return "failed"