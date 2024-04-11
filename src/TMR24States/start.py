import rospy
import smach
from std_msgs.msg import Empty

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])

    def execute(self, userdata):
        rospy.loginfo("START state executing")
        takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            if takeoff_pub.get_num_connections() > 0:
                break
            rospy.loginfo("Waiting for drone to be connected")
            rospy.Rate(1).sleep()

        if not rospy.is_shutdown():
            rospy.loginfo("Sending Take Off")
            takeoff_pub.publish(Empty())

            rospy.loginfo("Waiting for take off to be completed")
            rospy.sleep(10)

            return "succeeded"
        else:
            return "failed"