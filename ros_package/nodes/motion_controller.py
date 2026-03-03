#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msg.msg import JointState
from window_cleaner.msg import MotorCommand, RobotState, Stop

class MotionController:
    def __init__(self):
        rospy.init_node('motion_controller')

        # State
        self.target = None
        self.stop_active = False
        self.last_stop_time = rospy.Time.now()

        # Publishers
        self.motor_pub = rospy.Publisher('/motor_commands', MotorCommand, queue_size=10)
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)

        # Subscribers
        rospy.Subscriber('/target_position', Point, self.target_cb)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)
        rospy.Subscriber('/emergency_stop', Stop, self.stop_cb)

        # State publish timer
        rospy.Timer(rospy.Duration(0.1), self.publish_state)
        rospy.loginfo("Motion Controller initialized")

    def target_cb(self, msg):
        self.target = msg
        rospy.loginfo(f"Received target position: {self.target}")

        # Cable geometry and motor command calculations?

    def joint_state_cb(self, msg):
        # Process joint states for feedback control
        pass

    def stop_cb(self, msg):
        self.stop_active = msg.active
        if self.stop_active:
            self.last_stop_time = rospy.Time.now()
            rospy.logwarn(f"Emergency stop activated! - {msg.reason}")
        else:
            rospy.loginfo("Emergency stop deactivated")

    def publish_state(self, event):
        state = RobotState()
        state.header.stamp = rospy.Time.now()
        state.status = RobotState.STATUS_STOP if self.stop_active else RobotState.STATUS_ACTIVE

        # Publish position from encoder states
        self.state_pub.publish(state)

if __name__ == '__main__':
    try:
        MotionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass