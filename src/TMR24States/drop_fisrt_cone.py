import rospy
import smach
import requests
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class DropFirstCone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])

    def execute(self, userdata):
        rospy.loginfo("DROP FIRST CONE state executing")
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        rospy.sleep(1)

        if not rospy.is_shutdown():
        
            rospy.loginfo("DROPPING FIRST CONE !")
            try:
                msg = Twist()
                msg.linear.z = -0.3
                msg.linear.y = -0.1
                msg.linear.x = 0.07
                movement_pub.publish(msg)
                rospy.sleep(2)
                movement_pub.publish(Twist())

                respuesta = requests.get("http://192.168.42.100/encender")
                rospy.loginfo("Cone droped")
                rospy.sleep(3)

                msg = Twist()
                msg.linear.y = 0.1
                movement_pub.publish(msg)
                rospy.sleep(2)

                land_pub.publish(Empty())
                return "succeeded"
            except:
                rospy.loginfo("Error al enviar comando a ESP32")

                msg = Twist()
                msg.linear.y = 0.05
                movement_pub.publish(msg)
                rospy.sleep(2)

                return "failed"

        return "failed"