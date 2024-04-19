import rospy 
import smach
from TMR24States.start import Start
from TMR24States.go_to_aruco import GoToAruco
from TMR24States.grab_cone import GrabCone
from TMR24States.follow_line import FollowLine
from TMR24States.cross_window import CrossWindow
from TMR24States.drop_cone import DropCone
from TMR24States.finish import Finish
from TMR24States.aling_to_aruco import AlignToAruco

if __name__ == "__main__":
    try : 
        rospy.init_node("state_machine")
        rospy.loginfo("Starting state machine node") 
        sm = smach.StateMachine(outcomes=["completed"])

        with sm:
            smach.StateMachine.add( "START", Start(), 
                                   transitions={"succeeded":"CROSSWINDOW",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "GOTOARUCO_0", GoToAruco(0, 200, -45), 
                                   transitions={"succeeded":"GOTOARUCO_200",
                                                "skipped":"GRABCONE",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "GOTOARUCO_200", GoToAruco(200, -1, -45), 
                                   transitions={"succeeded":"GRABCONE",
                                                "skipped":"FINISH",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "GRABCONE", GrabCone(), 
                                   transitions={"succeeded":"GOTOARUCO_600",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "GOTOARUCO_600", GoToAruco(600, -1, -45), 
                                   transitions={"succeeded":"ALIGNTOARUCO_600",
                                                "skipped":"FINISH",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "ALIGNTOARUCO_600", AlignToAruco(600, 90), 
                                   transitions={"succeeded":"FOLLOWLINE_1",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "FOLLOWLINE_1", FollowLine(), 
                                   transitions={"succeeded":"CROSSWINDOW",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "CROSSWINDOW", CrossWindow(), 
                                   transitions={"succeeded":"GOTOARUCO_900",
                                                "failed":"FINISH"} )

            #smach.StateMachine.add( "FOLLOWLINE_2", FollowLine(), 
            #                       transitions={"succeeded":"GOTOARUCO_900",
            #                                    "failed":"FINISH"} )
            
            smach.StateMachine.add( "GOTOARUCO_900", GoToAruco(900, -1, -45), 
                                   transitions={"succeeded":"DROPCONE",
                                                "skipped":"FINISH",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "DROPCONE", DropCone(), 
                                   transitions={"succeeded":"FINISH",
                                                "failed":"FINISH"} )
            
            smach.StateMachine.add( "FINISH", Finish(), 
                                   transitions={"ended":"completed"} )

        result = sm.execute()

        rospy.loginfo("State machine node ended with result : ", result)
    except rospy.ROSInterruptException:
        pass