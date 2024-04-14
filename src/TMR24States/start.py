import rospy
import smach
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])

    def execute(self, userdata):
        rospy.loginfo("START state executing")
        takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            if takeoff_pub.get_num_connections() > 0:
                break
            rospy.loginfo("Waiting for drone to be connected")
            rospy.sleep(1)

        if not rospy.is_shutdown():
            rospy.loginfo("Sending Take Off")
            takeoff_pub.publish(Empty())

            rospy.loginfo("Waiting for take off to be completed")
            rospy.sleep(10)
            msg=Twist()
            msg.linear.z = 0.1
            movement_pub.publish(msg)
            rospy.sleep(5)
            movement_pub.publish(Twist())

            return "succeeded"
        else:
            return "failed"