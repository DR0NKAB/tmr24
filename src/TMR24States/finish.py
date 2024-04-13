import rospy
import smach
from std_msgs.msg import Empty

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["ended"])

    def execute(self, userdata):
        rospy.loginfo("FINISH state executing")
        land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        rospy.sleep(1)

        rospy.loginfo("Sending continuous land messages")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            land_pub.publish(Empty())
            rate.sleep()

        return "ended"