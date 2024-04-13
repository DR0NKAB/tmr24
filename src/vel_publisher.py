import rospy
import math
import pygame.joystick
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

current_msg = None

def callback(message):
    global current_msg
    rospy.loginfo(f"Velocity setpoint changed to : \n{message}")
    current_msg = message

def vel_publisher():

    rospy.init_node("vel_publisher", anonymous=False)
    rospy.loginfo("Iniciando nodo publicador de velocidades")
    land_pub = rospy.Publisher("bebop/land", Empty, queue_size=10)
    takeoff_pub = rospy.Publisher("bebop/takeoff", Empty, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
    vel_sub = rospy.Subscriber("/vel_publisher/set_vel", Twist, callback)
    rospy.sleep(1)

    rospy.loginfo("Iniciando joystick")
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    quiet_zone = 0.4 # any negative number = no quiet zone and always override input from nodes
    
    rospy.loginfo("Nodo publicador de velocidades iniciado")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        override = False
        for event in pygame.event.get():
            continue
        axis_values = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        if abs(axis_values[0]) > quiet_zone or abs(axis_values[1]) > quiet_zone or abs(axis_values[3]) > quiet_zone or abs(axis_values[4]) > quiet_zone:
            override = True

        if button_states[0] == 1:
            rospy.loginfo("Sending hover and forced land")
            cmd_vel_pub.publish(Twist())
            land_pub.publish(Empty())
        elif button_states[3] == 1:
            rospy.loginfo("Sending forced takeoff and hover")
            cmd_vel_pub.publish(Twist())
            takeoff_pub.publish(Empty())
        elif button_states[1] == 1:
            rospy.loginfo("Sending forced hover")
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.z = 0
            cmd_vel_pub.publish(Twist())
        elif override:
            rospy.loginfo("Forcing manual input")
            msg = Twist()
            msg.linear.x = -axis_values[4]
            msg.linear.y = -axis_values[3]
            msg.linear.z = -axis_values[1]
            msg.angular.z = -axis_values[0]
            cmd_vel_pub.publish(msg)
        else:
            if current_msg is not None:
                cmd_vel_pub.publish(current_msg)

        rate.sleep()

    rospy.loginfo("Deteniendo joystick")
    joystick.quit()
    pygame.joystick.quit()
    pygame.quit()

    rospy.loginfo("Nodo publicador de velocidades detenido")

if __name__ == "__main__":
    try:
        vel_publisher()
    except rospy.ROSInterruptException:
        pass