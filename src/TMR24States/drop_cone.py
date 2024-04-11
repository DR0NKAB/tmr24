import rospy
import smach

class DropCone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])

    def execute(self, userdata):
        rospy.loginfo("DROPCONE state executing")
        return "succeeded"