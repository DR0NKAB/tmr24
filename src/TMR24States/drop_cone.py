import rospy
import smach
import requests
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class DropCone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])

    def execute(self, userdata):
        rospy.loginfo("DROPCONE state executing")
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        rospy.sleep(1)

        if not rospy.is_shutdown():
        
            rospy.loginfo("DROPPING CONE !")
            try:
                """msg = Twist()
                msg.linear.x = 0.1
                movement_pub.publish(msg)
                rospy.sleep(1)
                movement_pub.publish(Twist())"""
                respuesta = requests.get("http://192.168.42.100/encender")
                rospy.loginfo("Cone droped")
                rospy.sleep(3)
                land_pub.publish(Empty())
                return "succeeded"
            except:
                rospy.loginfo("Error al enviar comando a ESP32")
                rospy.sleep(3)
                return "failed"

        return "failed"