import rospy
from std_msgs.msg import Empty

def take_off_and_land():

	rospy.init_node("take_off_and_land")
	rospy.loginfo("Iniciando nodo")

	take_off_publisher = rospy.Publisher("bebop/takeoff", Empty, queue_size=10)
	land_publisher = rospy.Publisher("bebop/land", Empty, queue_size=10)

	rospy.sleep(1)
	
	rospy.loginfo("Despegando")
	take_off_publisher.publish(Empty())
	
	rospy.loginfo("Esperando 10 segundos")
	rospy.sleep(10)

	rospy.loginfo("Aterrizando")
	land_publisher.publish(Empty())

	while not rospy.is_shutdown():
		rospy.loginfo("Terminado, esperando")
		rospy.sleep(1)

if __name__ == "__main__":
	try:
		take_off_and_land()
	except:
		print("ErorDK")
