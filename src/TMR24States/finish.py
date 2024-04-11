import rospy
import smach

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])

    def execute(self, userdata):
        rospy.loginfo("FINISH state executing")
        return "succeeded"