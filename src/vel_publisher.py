import rospy
from geometry_msgs.msg import Twist

current_msg = None

def callback(message):
    global current_msg
    rospy.loginfo(f"Velocity setpoint changed to : \n{message}")
    current_msg = message

def vel_publisher():

    rospy.init_node("vel_publisher", anonymous=False)
    rospy.loginfo("Iniciando nodo publicador de velocidades")
    cmd_vel_publisher = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
    vel_sub = rospy.Subscriber("/vel_publisher/set_vel", Twist, callback)
    rospy.sleep(1)
    
    rospy.loginfo("Nodo publicador de velocidades iniciado")
    while not rospy.is_shutdown():
        if current_msg is not None:
            cmd_vel_publisher.publish(current_msg)
            rospy.Rate(30).sleep()

    rospy.loginfo("Nodo publicador de velocidades detenido")

if __name__ == "__main__":
    try:
        vel_publisher()
    except rospy.ROSInterruptException:
        pass