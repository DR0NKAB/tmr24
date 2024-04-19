import rospy
import smach
import requests
from tmr24.msg import Cone
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class DropCone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.horizontal_error = None
        self.vertical_error = None
    
    def callback(self, message):
        self.horizontal_error = message.horizontal_error
        self.vertical_error = message.vertical_error

    def execute(self, userdata):
        rospy.loginfo("DROPCONE state executing")
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        cone_sub = rospy.Subscriber("/cone_detect/cones", Cone, self.callback)
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
        sampling_time = 0.1
        no_cone_conter = 0
        no_cone_limit_secs = 10
        tolerance = 20
        zero_error_conter = 0
        zero_error_limit = 50
        kp_v = 0.001
        kp_h = 0.001
        control_rate = rospy.Rate(1/sampling_time)
        while not rospy.is_shutdown():
            if self.horizontal_error != None and self.vertical_error != None :
                msg = Twist()
                msg.linear.y = -1 * kp_h * self.horizontal_error
                msg.linear.x = kp_v * self.vertical_error
                if self.horizontal_error < tolerance and self.vertical_error < tolerance:
                    zero_error_conter = zero_error_conter + 1
                    msg.linear.z = -0.05
                
                if zero_error_conter >= zero_error_limit:
                    rospy("Cone centered, leaving cone !")
                    comando = 'encender'
                    try:
                        respuesta = requests.get("http://192.168.16.103/" + comando)
                        rospy.loginfo("Cone droped")
                    except:
                        rospy.loginfo("Error al enviar comando a ESP32")

                    rospy.sleep(3)
                    land_pub.publish(Empty())
                    return "succeeded"

                movement_pub.publish(msg)
                control_rate.sleep()
            else:
                rospy.loginfo(f"No cone conter : {no_cone_conter}")
                if no_cone_conter > no_cone_limit_secs:
                    rospy.loginfo("No cone found, and time limit reached")
                    return "failed"
                else:
                    rospy.loginfo("Waiting for a cone to be detected")
                    rospy.sleep(1)
                    no_cone_conter = no_cone_conter + 1

        return "failed"