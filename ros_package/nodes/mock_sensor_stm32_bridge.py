#!/usr/bin/env python3
import rospy
from window_cleaner.msg import Stop

class MockSensorSTM32Bridge:
    def __init__(self):
        rospy.init_node('sensor_stm32_bridge')  # Same name as real node

        self.estop_pub = rospy.Publisher('/emergency_stop', Stop, queue_size=1, latch=True)

        # Publish safe default
        self._publish(active=False, reason="mock startup — all clear")

        # Timer to allow scripted obstacle injection via rosparam
        rospy.Timer(rospy.Duration(0.5), self.check_injected_obstacle)

        rospy.loginfo("MOCK sensor_stm32_bridge ready")

    def check_injected_obstacle(self, _event):
        # Set /mock_obstacle:=true on the param server to simulate an obstacle
        obstacle = rospy.get_param('mock_obstacle', False)
        self._publish(active=obstacle,
                      reason="mock obstacle injected" if obstacle else "mock all clear")

    def _publish(self, active, reason):
        msg = Stop()
        msg.header.stamp = rospy.Time.now()
        msg.active = active
        msg.reason = reason
        self.estop_pub.publish(msg)

if __name__ == '__main__':
    MockSensorSTM32Bridge()
    rospy.spin()