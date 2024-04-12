import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def movements():

    rospy.init_node("movements", anonymous=False)
    rospy.loginfo("Iniciando nodo")
    take_off_publisher = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
    cmd_vel_publisher = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
    land_publisher = rospy.Publisher("/bebop/land", Empty, queue_size=10)
    rospy.sleep(1)

    rospy.loginfo("Despegando")
    take_off_publisher.publish(Empty())

    rospy.loginfo("Hover")
    rospy.sleep(5)

    print("Moviendo")
    twist = Twist()
    twist.linear.z = 0.2
    cmd_vel_publisher.publish(twist)
    rospy.sleep(2)

    rospy.loginfo("Hover")
    cmd_vel_publisher.publish(Twist())
    rospy.sleep(5)

    rospy.loginfo("Aterrizando")
    land_publisher.publish(Empty())

if __name__ == "__main__":
    try:
        movements()
    except rospy.ROSInterruptException:
        pass