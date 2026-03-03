#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from window_cleaner.msg import RobotState, CleaningResult
from window_cleaner.srv import SetCleaningArea, SetCleaningAreaResponse
from window_cleaner.srv import AbortMission, AbortMissionResponse

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')

        # State
        self.robot_state = None
        self.waiting_for_result = False
        self.waypoints = []
        self.current_index = 0

        # Publishers
        self.target_pub = rospy.Publisher('/target_position', Point, queue_size=10)
        self.capture_pub = rospy.Publisher('/capture_request', Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/robot_state', RobotState, self.state_cb)
        rospy.Subscriber('/cleaning_result', CleaningResult, self.result_cb)

        # Services
        rospy.Service('set_cleaning_area', SetCleaningArea, self.handle_set_cleaning_area)
        rospy.Service('abort_mission', AbortMission, self.handle_abort_mission)

        rospy.loginfo("Navigation Node initialized")

    def state_cb(self, msg):
        self.robot_state = msg
        rospy.loginfo(f"Received robot state: {self.robot_state.status}")

    def result_cb(self, msg):
        if self.waiting_for_result:
            rospy.loginfo(f"Received cleaning result: {msg.success}")
            self.waiting_for_result = False
            # Retry logic and moving to next waypoint can be handled here

    def handle_set_cleaning_area(self, req):
        rospy.loginfo(f"navigation_node: SetCleaningArea called with: {req}")

        return SetCleaningAreaResponse(success=True, message="Cleaning area set successfully")
    
    def handle_abort_mission(self, req):
        rospy.loginfo("navigation_node: AbortMission called")
        self.waypoints = []
        self.current_index = 0
        self.waiting_for_result = False
        return AbortMissionResponse(success=True, message="Mission aborted successfully")
    
if __name__ == '__main__':
    try:
        NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass