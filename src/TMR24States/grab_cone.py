import rospy
import smach
from geometry_msgs.msg import Twist
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
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        continue_sub = rospy.Subscriber("/state_machine/continue_mission", Empty, self.callback)
        rospy.sleep(1)

        while not rospy.is_shutdown:
            if self.continue_mission:
                break
            rospy.loginfo("Waiting for message to continue")
            rospy.sleep(1)

        if not rospy.is_shutdown():
            rospy.loginfo("Sending land to drone")
            land_pub.publish(Empty())
            rospy.loginfo("Waiting for take off to be completed")
            rospy.sleep(10)

        if not rospy.is_shutdown():
            rospy.loginfo("Sending Takeoff to continue")
            takeoff_pub.publish(Empty())

            rospy.loginfo("Waiting for take off to be completed")
            rospy.sleep(10)

            rospy.loginfo("Going to desired height")
            msg=Twist()
            msg.linear.z = 0.3
            movement_pub.publish(msg)
            rospy.sleep(2)
            movement_pub.publish(Twist())

            return "succeeded"
        
        return "failed"
