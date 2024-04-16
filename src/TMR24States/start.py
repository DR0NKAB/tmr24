import rospy
import smach
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.start_mission = False

    def callback(self, message):
        self.start_mission = True

    def execute(self, userdata):
        rospy.loginfo("START state executing")
        takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        start_sub = rospy.Subscriber("/state_machine/continue_mission", Empty, self.callback)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            if self.start_mission:
                break
            rospy.loginfo("Waiting for Empty message to start")
            rospy.sleep(1)

        if not rospy.is_shutdown():
            rospy.loginfo("Sending Take Off")
            takeoff_pub.publish(Empty())

            rospy.loginfo("Waiting for take off to be completed")
            rospy.sleep(10)

            rospy.loginfo("Going to desired height")
            msg=Twist()
            msg.linear.z = 0.2
            movement_pub.publish(msg)
            rospy.sleep(2)
            movement_pub.publish(Twist())

            return "succeeded"
        else:
            return "failed"