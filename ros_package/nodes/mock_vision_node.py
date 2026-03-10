#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from window_cleaner.msg import CleaningResult

class MockVisionNode:
    def __init__(self):
        rospy.init_node('vision_node')  # Same name as real node

        self.result_pub = rospy.Publisher('/cleaning_result', CleaningResult, queue_size=1)
        rospy.Subscriber('/capture_request', Bool, self.capture_request_cb)

        rospy.loginfo("MOCK vision_node ready")

    def capture_request_cb(self, msg):
        if not msg.data:
            return
        rospy.sleep(0.5)  # Simulate processing time

        # Controllable via rosparam: set /mock_clean_success:=false to simulate failure
        success = rospy.get_param('mock_clean_success', True)

        result = CleaningResult()
        result.header.stamp = rospy.Time.now()
        result.clean_successful = success
        result.confidence = 0.95 if success else 0.85
        result.failure_reason = "" if success else "mock: simulated failure"
        self.result_pub.publish(result)
        rospy.loginfo(f"MOCK vision_node: published result success={success}")

if __name__ == '__main__':
    MockVisionNode()
    rospy.spin()