import rospy
import smach
from std_msgs.msg import Empty

class GrabCone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.continue_mission = False

    def callback(self, message):
        self.continue_mission = True

    def execute(self, userdata):
        rospy.loginfo("GRABCONE state executing")
        takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        continue_sub = rospy.Subscriber("/state_machine/continue_mission", Empty, self.callback)
        rospy.sleep(1)

        rospy.loginfo("Sending land to drone")
        land_pub.publish(Empty())

        while not rospy.is_shutdown:
            if self.continue_mission:
                break
            rospy.loginfo("Waiting for message to continue")
            rospy.sleep(1)

        if not rospy.is_shutdown():
            rospy.loginfo("Sending Takeoff to continue")
            takeoff_pub.publish(Empty())

            rospy.loginfo("Waiting for take off to be completed")
            rospy.sleep(10)
            return "succeeded"
        
        return "failed"
