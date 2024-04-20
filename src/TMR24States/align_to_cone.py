import rospy
import smach
from geometry_msgs.msg import Twist
from tmr24.msg import Cone

ancho_pantalla = 856
alto_pantalla = 450

class AlignToCone(smach.State):
    def __init__(self, align_yaw = True):
        smach.State.__init__(self, outcomes=["succeeded","failed"])
        self.current_horizontal_error = None
        self.current_vertical_error = None
        self.align_yaw = align_yaw
                
    def callback(self, message):
        self.current_horizontal_error = message.horizontal_error
        self.current_vertical_error = message.vertical_error

    def execute(self, userdata):
        rospy.loginfo(f"ALIGN TO CONE state executing")
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        movement_pub = rospy.Publisher("/vel_publisher/set_vel", Twist, queue_size=10)
        cone_sub = rospy.Subscriber("cone_detect/cones", Cone, self.callback)
        rospy.sleep(1)

        rospy.loginfo("Going to desired height")
        msg=Twist()
        msg.linear.z = -0.3
        movement_pub.publish(msg)
        rospy.sleep(2)
        movement_pub.publish(Twist())

        camera_angle = -90
        rospy.loginfo(f"Setting camera angle to {camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        sampling_time = 0.1

        rospy.loginfo(f"Entering XY control loop")
        kp_h_v = 0.0007
        kd_h_v = 0.001
        last_error_horizontal = 0
        last_error_vertical = 0
        
        counter_h = 0
        counter_v = 0
        tolerance_h_v = 20 #pixeles
        counter_h_limit = 30
        counter_v_limit = 30
        rate = rospy.Rate(1/sampling_time)

        while not rospy.is_shutdown():
            if self.current_vertical_error != None and self.current_horizontal_error != None:

                if abs(self.current_horizontal_error) < tolerance_h_v:
                    rospy.loginfo(f"Counter horizontal : {counter_h}")
                    counter_h = counter_h + 1 

                if abs(self.current_vertical_error) < tolerance_h_v:
                    rospy.loginfo(f"Counter vertical: {counter_v}")
                    counter_v = counter_v + 1 

                if counter_h > counter_h_limit and counter_v > counter_v_limit :
                    rospy.loginfo("Centrado en ejes XY, terminando bucle de control")
                    hover_msg = Twist()
                    movement_pub.publish(hover_msg)
                    return "succeeded"

                msg = Twist()
                msg.linear.x = kp_h_v * self.current_vertical_error
                msg.linear.y = kp_h_v * self.current_horizontal_error

                msg.linear.x = msg.linear.x + (kd_h_v * (self.current_vertical_error - last_error_vertical) / sampling_time)
                msg.linear.y = msg.linear.y + (kd_h_v * (self.current_horizontal_error - last_error_horizontal) / sampling_time)
                
                movement_pub.publish(msg)

                last_error_horizontal = self.current_horizontal_error
                last_error_vertical = self.current_vertical_error
            rate.sleep()

        return "failed"