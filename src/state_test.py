import rospy
import smach

from TMR24States.start import Start
from TMR24States.finish import Finish

from TMR24States.go_to_aruco import GoToAruco

if __name__ == "__main__":
    try : 
        rospy.init_node("state_machine")
        rospy.loginfo("Starting state machine node")
        sm = smach.StateMachine(outcomes=["completed"])

        with sm:
            
            smach.StateMachine.add( "START", Start(), 
                                   transitions={"succeeded":"CROSSWINDOW",
                                                "failed":"ERROR"} ) 
            
            smach.StateMachine.add( "CROSSWINDOW", GoToAruco(0,100, -30), 
                                   transitions={"succeeded":"FINISH",
                                                "skipped":"FINISH",
                                               "failed":"ERROR"} )
            smach.StateMachine.add( "FINISH", Finish(), 
                                   transitions={"ended":"completed"} )

            smach.StateMachine.add( "ERROR", Finish(), 
                                   transitions={"ended":"completed"} )
            

        result = sm.execute()
        rospy.loginfo("State machine node ended with result : ", result)
    except rospy.ROSInterruptException:
        pass