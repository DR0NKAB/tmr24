import rospy
import pygame.joystick
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

current_msg = None

def vel_publisher():

    rospy.init_node("vel_publisher", anonymous=False)
    rospy.loginfo("Xbox controller starting")
    land_pub = rospy.Publisher("bebop/land", Empty, queue_size=10)
    takeoff_pub = rospy.Publisher("bebop/takeoff", Empty, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
    rospy.sleep(1)

    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    rospy.loginfo("Manual controller started")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
    
        for event in pygame.event.get():
            continue
        
        axis_values = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

        if button_states[0] == 1:
            rospy.loginfo("Sending land")
            land_pub.publish(Empty())

        if button_states[3] == 1:
            rospy.loginfo("Sending takeoff")
            takeoff_pub.publish(Empty())

        msg = Twist()
        msg.linear.x = -axis_values[4]
        msg.linear.y = -axis_values[3]
        msg.linear.z = -axis_values[1]
        msg.angular.z = -axis_values[0]
        cmd_vel_pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Stopping manual controller")
    joystick.quit()
    pygame.joystick.quit()
    pygame.quit()

if __name__ == "__main__":
    try:
        vel_publisher()
    except rospy.ROSInterruptException:
        pass