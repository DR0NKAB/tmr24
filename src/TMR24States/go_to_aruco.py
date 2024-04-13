import rospy
import smach
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransform
from tmr24.msg import Aruco

error_yaw = 0
distance = 0
Kp = 1.0
Kd = 0.2
midpoint_x=frame_orig.shape[0]/2
negative_tolerance = midpoint_x - 20
positive_tolerance = midpoint_x + 20
zero_error_counter = 0


class GoToAruco(smach.State):
    def __init__(self, current_aruco_id, next_aruco_id, camera_angle):
        smach.State.__init__(self, outcomes=["succeeded", "skipped","failed"])
        self.camera_angle = camera_angle
        self.current_aruco_id = current_aruco_id
        self.next_aruco_id = next_aruco_id
        self.fiducial_transform = None

    def callback(self, message):
        id = message.id
        id = message.fiducial_id
        global error_yaw = message.error_yaw
        global distance = message.distance
        if id == self.current_aruco_id or id == self.next_aruco_id:
            self.fiducial_transform = message

    def execute(self, userdata):
        rospy.loginfo(f"GOTOARUCO state executing with aruco id : {self.current_aruco_id} skipping if aruco id : {self.next_aruco_id} is found")
        camera_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        movement_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        aruco_sub = rospy.Subscriber("/aruco_detect/detections", Aruco, self.callback)
        
        rospy.sleep(1)

        rospy.loginfo(f"Setting camera angle to {self.camera_angle}")
        camera_msg = Twist()
        camera_msg.angular.y = self.camera_angle
        camera_pub.publish(camera_msg)
        rospy.loginfo(f"Command was {camera_msg}")
        rospy.loginfo("Waiting for camera to be moved")
        rospy.sleep(3)

        rospy.loginfo(f"Searching for aruco {self.current_aruco_id} or {self.next_aruco_id}")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.fiducial_transform is not None:

                if error_yaw > negative_tolerance and error_yaw < positive_tolerance:
                    zero_error_counter = zero_error_counter + 1
                else
                    zero_error_counter = 0
                    movement_msg = Twist()
                    movement_msg.angular.z = Kp*error_yaw

                if zero_error_counter >= 500:
                    
                    aruco_sub.unregister()

                    rospy.loginfo(f"Found aruco {self.fiducial_transform.fiducial_id}")
                    
                    rospy.loginfo("Sending command to go to Aruco")
                    time_to_aruco = 5
                    gain_vel_to_twist = 0.1
                    #forward_dist = self.fiducial_transform.transform.translation.z
                    #left_dist = self.fiducial_transform.transform.translation.x
                    movement_msg = Twist()
                    movement_msg.linear.x = (distance / time_to_aruco) * gain_vel_to_twist 
                    #movement_msg.linear.y = (left_dist / time_to_aruco) * gain_vel_to_twist 
                    movement_pub.publish(movement_msg)
                    rospy.loginfo(f"Command was {movement_msg}")

                    rospy.loginfo("Waiting for command to be completed")
                    rospy.sleep(time_to_aruco)

                    if self.fiducial_transform.fiducial_id == self.current_aruco_id:
                        return "succeeded"
                    else :
                        return "skipped"

            else: 
                    movement_msg = Twist()
                    movement_msg.angular.z = 0.05

            rate.sleep()

            

        return "failed"