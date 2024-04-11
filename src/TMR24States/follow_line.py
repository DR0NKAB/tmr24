import rospy
import smach
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class FollowLine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.current_error = 0

    def callback(self, message):
        self.current_error = message.data

    def execute(self, userdata):
        rospy.loginfo("FOLLOWLINE state executing")
        movement_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        line_error_sub = rospy.Subscriber("/line_follower/line_error", Float64, self.callback)
        rospy.sleep(0.1)

        camera_angle = -90
        rospy.loginfo("Setting camera angle to ", camera_angle)
        camera_msg = Twist()
        camera_msg.angular.y = camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo("Command was : \n %s", str(camera_msg))
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        rospy.loginfo("Entering control loop")
        tiempo_muestreo = 0.1
        kp = 0.01
        while not rospy.is_shutdown():
            if not math.isnan(self.current_error):
                rospy.loginfo("Sending control command")
                movement_msg = Twist()
                movement_msg.linear.y = kp * self.current_error
                movement_pub.publish(movement_msg)
                rospy.loginfo("Command was : \n %s", str(movement_msg))
            else:
                movement_pub.publish(Twist())
                return "succeeded"
            rospy.sleep(tiempo_muestreo)

        return "failed"